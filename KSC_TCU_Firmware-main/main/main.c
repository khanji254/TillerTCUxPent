#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TAG "TCU_CURRENT"
#define AP_SSID "TCU-CURRENT"
#define AP_PASSWORD "tcu-current"
#define CURRENT_CHANNEL ADC_CHANNEL_4
#define REFERENCE_CHANNEL ADC_CHANNEL_3
#define SAMPLE_COUNT 64
#define LIVE_AVERAGE_COUNT 8

typedef struct {
    float vref_mv;
    float gain;
    float shunt_ohm;
    float scale;
    float offset_a;
} calibration_t;

static adc_oneshot_unit_handle_t adc_unit;
static adc_oneshot_unit_handle_t adc2_unit;
static adc_cali_handle_t adc_calibration;
static adc_cali_handle_t adc2_calibration;
static bool adc_calibration_valid;
static bool adc2_calibration_valid;
static calibration_t calibration = {
    .vref_mv = 1650.0f,
    .gain = 10.0f,
    .shunt_ohm = 0.00050f,
    .scale = 0.97f,
    .offset_a = 5.3f,
};

static esp_err_t load_calibration(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("current", NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    size_t size = sizeof(calibration);
    err = nvs_get_blob(nvs, "calibration", &calibration, &size);
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    return err;
}

static esp_err_t save_calibration(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("current", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs, "calibration", &calibration, sizeof(calibration));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

static bool calibration_is_valid(const calibration_t *value)
{
    return value->vref_mv >= 0.0f && value->vref_mv <= 3300.0f &&
           value->gain >= 0.1f && value->gain <= 1000.0f &&
           value->shunt_ohm >= 0.0000001f && value->shunt_ohm <= 1.0f &&
           value->scale >= -10.0f && value->scale <= 10.0f &&
           fabsf(value->scale) >= 0.001f &&
           value->offset_a >= -2000.0f && value->offset_a <= 2000.0f;
}

typedef struct {
    float mean_mv;
    float std_mv;
    float mean_raw;
    int good_samples;
} channel_stats_t;

typedef struct {
    channel_stats_t current;
    channel_stats_t reference;
    float current_a;
    float raw_current_a;
    int requested_samples;
    bool valid;
    bool reference_valid;
} reading_t;

typedef struct {
    float values[LIVE_AVERAGE_COUNT];
    size_t count;
    size_t next;
    double sum;
} live_average_t;

static live_average_t dashboard_average;
static live_average_t serial_average;
static portMUX_TYPE live_average_lock = portMUX_INITIALIZER_UNLOCKED;

static void reset_live_averages(void)
{
    portENTER_CRITICAL(&live_average_lock);
    memset(&dashboard_average, 0, sizeof(dashboard_average));
    memset(&serial_average, 0, sizeof(serial_average));
    portEXIT_CRITICAL(&live_average_lock);
}

static float update_live_average(live_average_t *average, float value)
{
    float result;

    portENTER_CRITICAL(&live_average_lock);
    if (average->count < LIVE_AVERAGE_COUNT) {
        average->values[average->count++] = value;
    } else {
        average->sum -= average->values[average->next];
        average->values[average->next] = value;
    }
    average->sum += value;
    average->next = (average->next + 1) % LIVE_AVERAGE_COUNT;
    result = (float)(average->sum / average->count);
    portEXIT_CRITICAL(&live_average_lock);
    return result;
}

static channel_stats_t finish_stats(double sum_mv, double sum_sq_mv,
                                    double sum_raw, int good)
{
    channel_stats_t stats = {0};
    stats.good_samples = good;
    if (good > 0) {
        double mean = sum_mv / good;
        double variance = (sum_sq_mv / good) - (mean * mean);
        stats.mean_mv = (float)mean;
        stats.std_mv = (float)(variance > 0.0 ? sqrt(variance) : 0.0);
        stats.mean_raw = (float)(sum_raw / good);
    }
    return stats;
}

static esp_err_t read_channel(adc_oneshot_unit_handle_t unit,
                              adc_cali_handle_t calibration_handle,
                              bool calibration_valid, adc_channel_t channel,
                              int samples, channel_stats_t *result,
                              bool drop_zero)
{
    double sum_mv = 0.0;
    double sum_sq_mv = 0.0;
    double sum_raw = 0.0;
    int good = 0;
    for (int i = 0; i < samples; ++i) {
        int sample;
        esp_err_t err = adc_oneshot_read(unit, channel, &sample);
        if (err != ESP_OK) {
            continue;
        }
        int millivolts;
        if (calibration_valid) {
            err = adc_cali_raw_to_voltage(calibration_handle, sample,
                                          &millivolts);
            if (err != ESP_OK) {
                continue;
            }
        } else {
            millivolts = sample * 3100 / 4095;
        }
        if (drop_zero && (millivolts <= 0 || sample <= 0)) {
            continue;
        }
        sum_mv += millivolts;
        sum_sq_mv += (double)millivolts * millivolts;
        sum_raw += sample;
        ++good;
    }
    *result = finish_stats(sum_mv, sum_sq_mv, sum_raw, good);
    return good > 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t take_reading(int samples, reading_t *reading)
{
    memset(reading, 0, sizeof(*reading));
    reading->requested_samples = samples;
    esp_err_t current_err =
        read_channel(adc_unit, adc_calibration, adc_calibration_valid,
                     CURRENT_CHANNEL, samples, &reading->current, false);
    esp_err_t reference_err =
        read_channel(adc2_unit, adc2_calibration, adc2_calibration_valid,
                     REFERENCE_CHANNEL, samples, &reading->reference, true);
    reading->valid = current_err == ESP_OK;
    reading->reference_valid =
        reference_err == ESP_OK && reading->reference.good_samples >= samples / 2;
    if (!reading->valid) {
        return current_err;
    }

    reading->raw_current_a =
        (reading->current.mean_mv - calibration.vref_mv) /
        (calibration.gain * calibration.shunt_ohm * 1000.0f);
    reading->current_a = fabsf(
        reading->raw_current_a * calibration.scale + calibration.offset_a);
    return ESP_OK;
}

static esp_err_t root_handler(httpd_req_t *request)
{
    static const char page[] =
        "<!doctype html><html><head><meta name=\"viewport\" content=\"width=device-width\">"
        "<title>TCU Current Calibration</title><style>"
        "body{margin:0;padding:16px;background:#101820;color:white;font:15px Arial}"
        ".wrap{max-width:980px;margin:auto}.card{margin-bottom:16px;padding:18px;"
        "border-radius:14px;background:#1e2b36}.grid{display:grid;"
        "grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px}"
        ".details{padding:12px;background:#101820;border-radius:8px;line-height:1.8}"
        "h1{text-align:center}.current{text-align:center;font-size:56px;color:#45d483}"
        "h2{color:#e5a83b;font-size:18px}input{width:100%;box-sizing:border-box;"
        "padding:8px;margin-top:4px;background:#101820;color:white;border:1px solid #456}"
        "button{margin:8px 8px 0 0;padding:10px 14px;border:0;border-radius:7px;"
        "background:#e5a83b;cursor:pointer}button.danger{background:#c9534f;color:white}"
        "table{border-collapse:collapse;width:100%;font-size:12px}th,td{padding:5px;"
        "border-bottom:1px solid #456;white-space:nowrap;text-align:right}.scroll{overflow-x:auto}"
        ".muted{color:#9aa;font-size:12px}</style></head><body><div class=\"wrap\">"
        "<div class=\"card\"><h1>TCU Current Calibration</h1><div id=\"current\" class=\"current\">--.---</div>"
        "<div style=\"text-align:center\">amperes (corrected) <span id=\"status\">Connecting...</span></div>"
        "<button onclick=\"zeroCurrent()\">Zero Current</button>"
        "<div class=\"muted\">Only use zero calibration when actual current is zero.</div></div>"
        "<div class=\"card\"><h2>Live raw data</h2><div class=\"grid\">"
        "<div class=\"details\">Current IO5<br>ADC: <span id=\"adc\">--</span> mV<br>"
        "Noise: <span id=\"adcStd\">--</span> mV<br>Raw ADC: <span id=\"rawAdc\">--</span><br>"
        "Raw current: <span id=\"rawCurrent\">--</span> A<br>Offset: <span id=\"offset\">--</span> A</div>"
        "<div class=\"details\">Reference IO14 (display only)<br>Voltage: <span id=\"refMv\">--</span> mV<br>"
        "Noise: <span id=\"refStd\">--</span> mV<br>Raw ADC: <span id=\"refRaw\">--</span><br>"
        "Measured - constant: <span id=\"refDelta\">--</span> mV<br>Vadc - reference: <span id=\"adcMinusRef\">--</span> mV</div>"
        "<div class=\"details\">System<br>Clients: <span id=\"clients\">--</span><br>"
        "Uptime: <span id=\"uptime\">--</span> s<br>Samples: <span id=\"samples\">--</span></div></div></div>"
        "<div class=\"card\"><h2>Constants</h2><div class=\"grid\">"
        "<label>INA gain<input id=\"gain\" type=\"number\" step=\"any\"></label>"
        "<label>Shunt (micro-ohms)<input id=\"shunt\" type=\"number\" step=\"any\"></label>"
        "<label>Reference constant (mV)<input id=\"vref\" type=\"number\" step=\"any\"></label>"
        "<label>Gain correction<input id=\"corr\" type=\"number\" step=\"any\"></label>"
        "<label>Offset (A)<input id=\"offsetCfg\" type=\"number\" step=\"any\"></label></div>"
        "<button onclick=\"applyConfig()\">Apply</button><button onclick=\"resetConfig()\">Reset defaults</button></div>"
        "<div class=\"card\"><h2>Capture calibration point</h2>"
        "<p>Control shunt voltage (mV): <input id=\"ctrlV\" type=\"number\" step=\"any\"></p>"
        "<p>Control shunt resistance (micro-ohms): <input id=\"ctrlR\" value=\"250\" type=\"number\" step=\"any\"></p>"
        "<p>Or override true current (A): <input id=\"override\" type=\"number\" step=\"any\"></p>"
        "<p>Capture samples: <input id=\"captureN\" value=\"1000\" type=\"number\" min=\"100\" max=\"4000\"></p>"
        "<button onclick=\"capture()\">Capture point</button><span id=\"trueA\"></span>"
        "<p class=\"muted\">Capture zero and several known currents. The fit uses Vadc = a + b * Itrue.</p></div>"
        "<div class=\"card\"><h2>Fit</h2><div id=\"fit\">Need at least two points at different currents.</div>"
        "<button id=\"applyFit\" onclick=\"applyFit()\" disabled>Apply fitted correction + offset</button></div>"
        "<div class=\"card\"><h2>Captured points</h2><div class=\"scroll\"><table><thead><tr>"
        "<th>#</th><th>True A</th><th>Ctrl mV</th><th>ADC mV</th><th>Std</th><th>Raw I</th><th>Device I</th><th>Fit err</th><th></th>"
        "</tr></thead><tbody id=\"rows\"></tbody></table></div><button onclick=\"exportCsv()\">Export CSV</button>"
        "<button class=\"danger\" onclick=\"clearPoints()\">Clear all</button></div></div><script>"
        "const $=id=>document.getElementById(id), f=(v,d)=>Number.isFinite(Number(v))?Number(v).toFixed(d):'--';"
        "let points=JSON.parse(localStorage.getItem('tcuCalPoints')||'[]'),fit=null,configDirty=false;"
        "['gain','shunt','vref','corr','offsetCfg'].forEach(id=>$(id).addEventListener('input',()=>configDirty=true));"
        "function show(d){$('current').textContent=f(d.current_a,3);$('adc').textContent=f(d.adc_mv,2);"
        "$('adcStd').textContent=f(d.adc_std_mv,3);$('rawAdc').textContent=f(d.raw_adc,0);"
        "$('rawCurrent').textContent=f(d.raw_current_a,3);$('offset').textContent=f(d.offset_a,3);"
        "$('refMv').textContent=d.ref_valid?f(d.ref_mv,2):'read failed';$('refStd').textContent=f(d.ref_std_mv,3);"
        "$('refRaw').textContent=f(d.ref_raw,0);$('refDelta').textContent=f(d.ref_mv-d.ref_const_mv,2);"
        "$('adcMinusRef').textContent=f(d.adc_mv-d.ref_mv,2);$('clients').textContent=d.wifi_clients;"
        "$('uptime').textContent=f(d.uptime_ms/1000,0);$('samples').textContent=d.samples;"
        "if(!configDirty)for(const [id,v] of [['gain',d.gain],['shunt',d.shunt_ohms*1e6],['vref',d.ref_const_mv],['corr',d.gain_corr],['offsetCfg',d.offset_a]])$(id).value=v;"
        "$('status').textContent=d.valid?(d.ref_valid?'Live':'Live (IO14 reference unavailable)'):'Measurement error';renderFit()}"
        "async function update(){try{let r=await fetch('/api/current',{cache:'no-store'});show(await r.json())}catch(e){$('status').textContent='Connection error'}}"
        "async function zeroCurrent(){let d=await fetch('/api/zero',{method:'POST'}).then(r=>r.json());alert(d.success?'Zero offset updated':d.error||'Zero failed');if(d.success)configDirty=false;update()}"
        "async function applyConfig(extra={}){let p=new URLSearchParams({gain:$('gain').value,shunt_ohms:String(Number($('shunt').value)/1e6),ref_mv:$('vref').value,gain_corr:$('corr').value,offset_a:$('offsetCfg').value,...extra});let d=await fetch('/api/config?'+p,{method:'POST'}).then(r=>r.json());if(!d.success){alert(d.error||'Config rejected');return}configDirty=false;update()}"
        "function resetConfig(){configDirty=false;applyConfig({reset:1})}"
        "function trueCurrent(){let o=Number($('override').value);if($('override').value!=='')return Number.isFinite(o)?o:null;let v=Number($('ctrlV').value),r=Number($('ctrlR').value);return r>0&&Number.isFinite(v)?v/r*1000:null}"
        "async function capture(){let t=trueCurrent();if(t===null){alert('Enter a control-shunt value or override current');return}let d=await fetch('/api/capture?n='+$('captureN').value).then(r=>r.json());if(!d.valid){alert('Capture failed');return}points.push({id:Date.now(),true_a:t,ctrl_mv:$('ctrlV').value,adc_mv:d.adc_mv,adc_std_mv:d.adc_std_mv,raw_current_a:d.raw_current_a,current_a:d.current_a});localStorage.setItem('tcuCalPoints',JSON.stringify(points));renderFit()}"
        "function linear(){if(points.length<2)return null;let mx=points.reduce((s,p)=>s+p.true_a,0)/points.length,my=points.reduce((s,p)=>s+p.adc_mv,0)/points.length,sxx=0,sxy=0,syy=0;points.forEach(p=>{let x=p.true_a-mx,y=p.adc_mv-my;sxx+=x*x;sxy+=x*y;syy+=y*y});if(sxx<1e-12)return null;let b=sxy/sxx,a=my-b*mx,res=points.reduce((s,p)=>s+Math.pow(p.adc_mv-(a+b*p.true_a),2),0);return{a:a,b:b,corr:(b*calibrationSensitivity())?1:1,rms:Math.sqrt(res/points.length)/Math.abs(b)}}"
        "function calibrationSensitivity(){return Number($('gain').value)*Number($('shunt').value)*1e-6*1000}"
        "function renderFit(){fit=linear();$('fit').textContent=fit?'Vadc = '+fit.a.toFixed(3)+' + '+fit.b.toFixed(4)+' * Itrue; RMS = '+fit.rms.toFixed(4)+' A':'Need at least two points at different currents.';$('applyFit').disabled=!fit;if(!fit){$('rows').innerHTML='';return}let sens=calibrationSensitivity(),corr=sens/fit.b,off=(fit.a-Number($('vref').value))/sens*corr*-1;$('fit').textContent+='; suggested correction = '+corr.toFixed(6)+', offset = '+off.toFixed(6)+' A';$('rows').innerHTML=points.map((p,i)=>'<tr><td>'+(i+1)+'</td><td>'+f(p.true_a,4)+'</td><td>'+f(p.ctrl_mv,3)+'</td><td>'+f(p.adc_mv,3)+'</td><td>'+f(p.adc_std_mv,3)+'</td><td>'+f(p.raw_current_a,3)+'</td><td>'+f(p.current_a,3)+'</td><td>'+f((p.adc_mv-(fit.a+fit.b*p.true_a))/fit.b,3)+'</td><td><button onclick=\"points.splice('+i+',1);renderFit()\">x</button></td></tr>').join('')}"
        "function applyFit(){if(!fit)return;let sens=calibrationSensitivity(),corr=sens/fit.b,off=(fit.a-Number($('vref').value))/sens*corr*-1;applyConfig({gain_corr:corr,offset_a:off})}"
        "function clearPoints(){if(confirm('Delete all captured points?')){points=[];localStorage.removeItem('tcuCalPoints');renderFit()}}"
        "function exportCsv(){let csv='true_a,ctrl_mv,adc_mv,adc_std_mv,raw_current_a,current_a\\n'+points.map(p=>[p.true_a,p.ctrl_mv,p.adc_mv,p.adc_std_mv,p.raw_current_a,p.current_a].join(',')).join('\\n'),a=document.createElement('a');a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));a.download='tcu_current_calibration.csv';a.click()}"
        "update();setInterval(update,500);</script></body></html>";
    httpd_resp_set_type(request, "text/html");
    return httpd_resp_send(request, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t current_handler(httpd_req_t *request)
{
    reading_t reading;
    esp_err_t err = take_reading(SAMPLE_COUNT, &reading);
    float displayed_current =
        err == ESP_OK ? update_live_average(&dashboard_average, reading.current_a)
                      : 0.0f;
    wifi_sta_list_t stations = {0};
    esp_wifi_ap_get_sta_list(&stations);
    char response[720];
    int length = snprintf(
        response, sizeof(response),
        "{\"valid\":%s,\"ref_valid\":%s,\"current_a\":%.6f,"
        "\"raw_current_a\":%.6f,\"adc_mv\":%.3f,\"adc_std_mv\":%.3f,"
        "\"raw_adc\":%d,\"ref_mv\":%.3f,\"ref_std_mv\":%.3f,\"ref_raw\":%d,"
        "\"samples\":%d,\"wifi_clients\":%d,\"uptime_ms\":%llu,"
        "\"gain\":%.6f,\"shunt_ohms\":%.9f,\"ref_const_mv\":%.3f,"
        "\"gain_corr\":%.6f,\"offset_a\":%.6f,\"sens_mv_per_a\":%.6f}",
        err == ESP_OK ? "true" : "false",
        reading.reference_valid ? "true" : "false", displayed_current,
        reading.raw_current_a, reading.current.mean_mv, reading.current.std_mv,
        (int)lroundf(reading.current.mean_raw), reading.reference.mean_mv,
        reading.reference.std_mv, (int)lroundf(reading.reference.mean_raw),
        reading.requested_samples, stations.num,
        (unsigned long long)(esp_log_timestamp()),
        calibration.gain, calibration.shunt_ohm, calibration.vref_mv,
        calibration.scale, calibration.offset_a,
        calibration.gain * calibration.shunt_ohm * 1000.0f);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, response, length);
}

static esp_err_t capture_handler(httpd_req_t *request)
{
    char query[64];
    char text[16];
    int samples = 1000;
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "n", text, sizeof(text)) == ESP_OK) {
        samples = atoi(text);
    }
    if (samples < 100) {
        samples = 100;
    } else if (samples > 4000) {
        samples = 4000;
    }
    reading_t reading;
    esp_err_t err = take_reading(samples, &reading);
    char response[720];
    int length = snprintf(
        response, sizeof(response),
        "{\"valid\":%s,\"ref_valid\":%s,\"current_a\":%.6f,"
        "\"raw_current_a\":%.6f,\"adc_mv\":%.3f,\"adc_std_mv\":%.3f,"
        "\"raw_adc\":%d,\"ref_mv\":%.3f,\"ref_std_mv\":%.3f,\"ref_raw\":%d,"
        "\"samples\":%d,\"gain\":%.6f,\"shunt_ohms\":%.9f,\"ref_const_mv\":%.3f,"
        "\"gain_corr\":%.6f,\"offset_a\":%.6f}",
        err == ESP_OK ? "true" : "false",
        reading.reference_valid ? "true" : "false", reading.current_a,
        reading.raw_current_a, reading.current.mean_mv, reading.current.std_mv,
        (int)lroundf(reading.current.mean_raw), reading.reference.mean_mv,
        reading.reference.std_mv, (int)lroundf(reading.reference.mean_raw),
        reading.requested_samples, calibration.gain, calibration.shunt_ohm,
        calibration.vref_mv, calibration.scale, calibration.offset_a);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, response, length);
}

static bool query_float(httpd_req_t *request, const char *key, float *value)
{
    char query[256];
    char text[32];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, key, text, sizeof(text)) != ESP_OK) {
        return false;
    }
    char *end;
    float parsed = strtof(text, &end);
    if (end == text || *end != '\0') {
        return false;
    }
    *value = parsed;
    return true;
}

static bool query_present(httpd_req_t *request, const char *key)
{
    char query[256];
    char value[8];
    return httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK &&
           httpd_query_key_value(query, key, value, sizeof(value)) == ESP_OK;
}

static esp_err_t config_handler(httpd_req_t *request)
{
    calibration_t candidate = calibration;
    if (query_present(request, "reset")) {
        candidate = (calibration_t){
            .vref_mv = 1650.0f,
            .gain = 10.0f,
            .shunt_ohm = 0.00050f,
            .scale = 0.97f,
            .offset_a = 5.3f,
        };
    } else {
        float value;
        if (query_float(request, "ref_mv", &value)) {
            candidate.vref_mv = value;
        }
        if (query_float(request, "gain", &value)) {
            candidate.gain = value;
        }
        if (query_float(request, "shunt_ohms", &value)) {
            candidate.shunt_ohm = value;
        }
        if (query_float(request, "gain_corr", &value)) {
            candidate.scale = value;
        }
        if (query_float(request, "offset_a", &value)) {
            candidate.offset_a = value;
        }
    }
    if (!calibration_is_valid(&candidate)) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                            "Invalid calibration value");
        return ESP_OK;
    }
    calibration = candidate;
    esp_err_t err = save_calibration();
    if (err != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Calibration save failed");
        return ESP_OK;
    }
    reset_live_averages();
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, "{\"success\":true}");
}

static esp_err_t zero_handler(httpd_req_t *request)
{
    reading_t reading;
    if (take_reading(256, &reading) != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Measurement failed");
        return ESP_OK;
    }
    calibration.offset_a = -(reading.raw_current_a * calibration.scale);
    if (save_calibration() != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Calibration save failed");
        return ESP_OK;
    }
    reset_live_averages();
    char response[96];
    snprintf(response, sizeof(response), "{\"success\":true,\"offset_a\":%.6f}",
             calibration.offset_a);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, response);
}

static void start_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));

    wifi_config_t ap_config = {0};
    memcpy(ap_config.ap.ssid, AP_SSID, sizeof(AP_SSID) - 1);
    memcpy(ap_config.ap.password, AP_PASSWORD, sizeof(AP_PASSWORD) - 1);
    ap_config.ap.ssid_len = sizeof(AP_SSID) - 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    httpd_uri_t current = {
        .uri = "/api/current", .method = HTTP_GET, .handler = current_handler};
    httpd_uri_t capture = {
        .uri = "/api/capture", .method = HTTP_GET, .handler = capture_handler};
    httpd_uri_t config_uri = {
        .uri = "/api/config", .method = HTTP_POST, .handler = config_handler};
    httpd_uri_t zero = {
        .uri = "/api/zero", .method = HTTP_POST, .handler = zero_handler};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &current));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &capture));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &zero));
}

static void serial_measurement_task(void *argument)
{
    (void)argument;
    while (true) {
        reading_t reading;
        if (take_reading(SAMPLE_COUNT, &reading) == ESP_OK) {
            float displayed_current =
                update_live_average(&serial_average, reading.current_a);
            ESP_LOGI(TAG,
                     "Current: %.3f A average (raw %.3f A), ADC: %.2f mV, noise: %.3f mV",
                     displayed_current,
                     reading.raw_current_a,
                     reading.current.mean_mv, reading.current.std_mv);
        } else {
            ESP_LOGE(TAG, "Current measurement failed");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    if (load_calibration() != ESP_OK || !calibration_is_valid(&calibration)) {
        calibration = (calibration_t){
            .vref_mv = 1650.0f,
            .gain = 10.0f,
            .shunt_ohm = 0.00050f,
            .scale = 0.97f,
            .offset_a = 5.3f,
        };
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_unit));
    adc_oneshot_unit_init_cfg_t adc2_config = {
        .unit_id = ADC_UNIT_2, .ulp_mode = ADC_ULP_MODE_DISABLE};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_config, &adc2_unit));
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_unit, CURRENT_CHANNEL,
                                                &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_unit, REFERENCE_CHANNEL,
                                                &channel_config));

    adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_calibration_valid =
        adc_cali_create_scheme_curve_fitting(&calibration_config,
                                             &adc_calibration) == ESP_OK;
    adc_cali_curve_fitting_config_t adc2_calibration_config = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc2_calibration_valid =
        adc_cali_create_scheme_curve_fitting(&adc2_calibration_config,
                                             &adc2_calibration) == ESP_OK;

    start_wifi();
    start_http_server();
    xTaskCreate(serial_measurement_task, "serial_measurement", 4096, NULL, 5,
                NULL);
    ESP_LOGI(TAG, "AP ready: SSID=%s URL=http://192.168.4.1/", AP_SSID);
}
