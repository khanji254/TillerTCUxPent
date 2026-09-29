/**
 * @file bms_monitor_types.h
 * @brief Type definitions for the multi-battery BMS monitoring subsystem.
 *
 * Defines battery identifiers, status codes, the queued data structure
 * passed between the BMS monitor task and the MQTT publish task, and
 * system-wide statistics.
 *
 * Pin definitions previously held here have been moved to board_config.h.
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef BMS_MONITOR_TYPES_H_
#define BMS_MONITOR_TYPES_H_

#include <stdint.h>
#include <string.h>
#include "jk_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * QUEUE CONFIGURATION
 * ========================================================================= */

/** Number of JK-BMS batteries monitored — sourced from board_config.h */
#ifndef BMS_BATTERY_COUNT
//#define BMS_BATTERY_COUNT         1
#define BMS_BATTERY_COUNT           2
//#define BMS_BATTERY_COUNT         3
#endif

/** BMS monitoring cycle interval (ms).
 *  Actual cycle time is dominated by RS-485 read latency and retry logic;
 *  this value caps the minimum idle delay between full read cycles. */
#define BMS_READ_INTERVAL_MS      1000

/* =========================================================================
 * BATTERY IDENTIFIERS
 * ========================================================================= */

/**
 * @brief Logical battery index enumeration.
 */
typedef enum {
    BMS_BATTERY_1       = 0,
    BMS_BATTERY_2       = 1,
    BMS_BATTERY_3       = 2,
    BMS_BATTERY_INVALID = 0xFF,
} bms_battery_id_t;

/* =========================================================================
 * STATUS CODES
 * ========================================================================= */

/**
 * @brief Result of a single battery read attempt.
 */
typedef enum {
    BMS_STATUS_OK        = 0,  /**< All registers read successfully           */
    BMS_STATUS_TIMEOUT   = 1,  /**< RS-485 response timed out                 */
    BMS_STATUS_CRC_ERROR = 2,  /**< Modbus CRC mismatch                       */
    BMS_STATUS_COMM_ERROR= 3,  /**< Framing or protocol error                 */
    BMS_STATUS_NO_DATA   = 4,  /**< Serial number could not be read           */
} bms_read_status_t;

/* =========================================================================
 * QUEUED DATA STRUCTURE
 * ========================================================================= */

/**
 * @brief Complete data snapshot for one battery, queued by the BMS task
 *        and consumed by the MQTT publish task.
 *
 * All physical values are in SI base units with the following conventions:
 *   batt_v            — millivolts  (mV)
 *   batt_i            — milliamps   (mA), positive = discharge
 *   batt_power        — milliwatts  (mW), positive = discharge
 *   remaining_capacity— milliamp-hours (mAh)
 */
typedef struct {
    bms_battery_id_t  battery_id;        /**< Which battery slot (0–2)       */
    bms_read_status_t status;            /**< Outcome of the read cycle       */
    uint32_t          timestamp;         /**< System tick timestamp (ms)      */

    /* JK-BMS register values */
    uint32_t  device_address;
    uint32_t  batt_v;                    /**< Pack voltage (mV)               */
    int32_t   batt_i;                    /**< Pack current (mA)               */
    int32_t   batt_power;                /**< Pack power (mW)                 */
    int32_t   remaining_capacity;        /**< Remaining capacity (mAh)        */
    uint32_t  cell_count;
    uint32_t  cells_present;
    uint16_t  cells_diff;                /**< Max cell voltage difference (mV)*/
    uint8_t   soc;                       /**< State of charge (%)             */
    uint8_t   soh;                       /**< State of health (%)             */
    uint8_t   charge_stat;               /**< Charge MOSFET state             */
    uint8_t   discharge_stat;            /**< Discharge MOSFET state          */
    uint32_t  charge_cycles;             /**< Total charge cycle count        */
    uint32_t  alarms;                    /**< Alarm bitmask                   */
    char      model_no[32];              /**< BMS model string                */
    char      serial_no[32];             /**< BMS serial number string        */
} bms_queued_data_t;

/* =========================================================================
 * STATISTICS STRUCTURES
 * ========================================================================= */

/**
 * @brief Per-battery read statistics for health monitoring.
 */
typedef struct {
    uint32_t successful_reads;
    uint32_t failed_reads;
    uint32_t timeout_errors;
    uint32_t crc_errors;
    uint32_t last_success_timestamp;
    uint32_t last_failure_timestamp;
} bms_battery_stats_t;

/**
 * @brief System-wide BMS monitoring statistics.
 */
typedef struct {
    bms_battery_stats_t battery_stats[BMS_BATTERY_COUNT];
    uint32_t            queue_full_events;
    uint32_t            queue_overruns;
    uint32_t            total_readings;
} bms_system_stats_t;

#ifdef __cplusplus
}
#endif

#endif /* BMS_MONITOR_TYPES_H_ */