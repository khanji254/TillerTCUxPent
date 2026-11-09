/*
 * jk_bms.h
 *
 *  Created on: Jun 23, 2025
 *      Author: collo
 *      Addons: don
 */

#ifndef INC_JK_BMS_H_
#define INC_JK_BMS_H_

#include "modbus_rtu.h"

#define JK_MODBUS_REG_TOGGLE_CHARGE				0x1070
#define JK_MODBUS_REG_TOGGLE_DISCHARGE			0x1074

#define JK_MODBUS_REG_CELL_COUNT                0x106C
#define JK_MODBUS_REG_CELLS_PRESENT             0x1240
#define JK_MODBUS_REG_CELL_DIFF                 0x1246

#define JK_MODBUS_REG_BATT_VOLTAGE				0x1290
#define JK_MODBUS_REG_BATT_CURRENT				0x1298
#define JK_MODBUS_REG_BATT_POWER				0x1294
#define JK_MODBUS_REG_BATT_REMAINING_CAPACITY	0x12A8
#define JK_MODBUS_REG_BATT_SOC					0x12A6
#define JK_MODBUS_REG_BATT_SOH                  0x12B8
#define JK_MODBUS_REG_BATT_CYCLES				0x12B0
#define JK_MODBUS_REG_CHARGE_DISCHARGE_STAT		0x12C0
#define JK_MODBUS_REG_ALARMS					0x12A0
#define JK_MODBUS_REG_CELL_VOLTAGE              0X1200

#define JK_MODBUS_REG_DEVICE_ID					0x1400
#define JK_MODBUS_REG_DEVICE_ADDRESS            0x1108

typedef struct{
	modbus_rtu_dev_t modbus;
}jk_device_t;

typedef enum{
	JK_MOSFET_LEAVE_UNCHANGED = -1,
	JK_MOSFET_DISABLE = 0,
	JK_MOSFET_ENABLE = 1
}jk_mosfet_set_t;

int jk_bms_read_cells_present(jk_device_t *dev, uint32_t *dst);
int jk_bms_read_device_address(jk_device_t *dev, uint32_t *dst);
int jk_bms_read_cells_diff(jk_device_t *dev, uint16_t *dst);
int jk_bms_read_cell_count(jk_device_t *dev, uint32_t *dst);
int jk_bms_read_soc(jk_device_t *dev, uint8_t *dst);
int jk_bms_read_soh(jk_device_t *dev, uint8_t *dst);
int jk_bms_read_device_id(jk_device_t *dev, char *dst);
int jk_bms_read_batt_voltage(jk_device_t *dev, uint32_t *dst);
int jk_bms_read_batt_current(jk_device_t *dev, int32_t *dst);
int jk_bms_read_batt_power(jk_device_t *dev, uint32_t *dst);
int jk_bms_read_batt_remaining_capacity(jk_device_t *dev, int32_t *dst);
int jk_bms_read_charge_cycles(jk_device_t *dev, uint32_t *dst);
int jk_bms_read_charge_discharge_stat(jk_device_t *dev, uint8_t *charge, uint8_t *discharge);
int jk_bms_read_alarms(jk_device_t *dev, uint32_t *dst);

int jk_bms_toggle_charge(jk_device_t *dev, uint8_t state);
int jk_bms_toggle_discharge(jk_device_t *dev, uint8_t state);
int jk_bms_toggle_rs485(jk_device_t *dev, uint8_t state);
int jk_bms_read_cell_voltages(jk_device_t *dev, uint16_t *jk_cells);
int jk_bms_change_device_address(jk_device_t *dev, uint8_t dev_address);
int jk_bms_read_serial(jk_device_t *dev, char *serial_number);
int jk_bms_read_all(jk_device_t *dev);

#endif /* INC_JK_BMS_H_ */
