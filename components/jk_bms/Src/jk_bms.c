/*
 * jk_bms.c
 *
 *  Created on: Jun 23, 2025
 *      Author: collo
 *      Addons: don
 */


#include "jk_bms.h"
#include <string.h>

int jk_bms_read_device_address(jk_device_t *dev, uint32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_DEVICE_ADDRESS, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((uint32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}
int jk_bms_read_cells_present(jk_device_t *dev, uint32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_CELLS_PRESENT, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((uint32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}

int jk_bms_read_cells_diff(jk_device_t *dev, uint16_t *dst)
{
	int res;
	uint8_t _dst[2];
	res = modbus_rtu_read_reg(&dev->modbus, JK_MODBUS_REG_CELL_DIFF, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((uint16_t)_dst[0]) << 8;
		*dst |= _dst[1];
	}

	return res;
}

int jk_bms_read_cell_count(jk_device_t *dev, uint32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_CELL_COUNT, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((uint32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}

int jk_bms_read_soc(jk_device_t *dev, uint8_t *dst)
{
	int res;
	uint8_t _dst[2];
	res = modbus_rtu_read_reg(&dev->modbus, JK_MODBUS_REG_BATT_SOC, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = _dst[1];
	}

	return res;
}

int jk_bms_read_soh(jk_device_t *dev, uint8_t *dst)
{
	int res;
	uint8_t _dst[2];
	res = modbus_rtu_read_reg(&dev->modbus, JK_MODBUS_REG_BATT_SOH, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = _dst[0];
	}

	return res;
}

int jk_bms_read_device_id(jk_device_t *dev, char *dst)
{
	int res;
	uint8_t _dst[16];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_DEVICE_ID, (16/2), _dst);
	if(res == MODBUS_RTU_OK){
		memcpy(dst, _dst, sizeof(_dst));
	}

	return res;
}

int jk_bms_read_batt_voltage(jk_device_t *dev, uint32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_BATT_VOLTAGE, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((uint32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}

int jk_bms_read_batt_current(jk_device_t *dev, int32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_BATT_CURRENT, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((int32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}

int jk_bms_read_batt_power(jk_device_t *dev, uint32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_BATT_POWER, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((uint32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}

int jk_bms_read_batt_remaining_capacity(jk_device_t *dev, int32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_BATT_REMAINING_CAPACITY, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((int32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}

int jk_bms_read_charge_cycles(jk_device_t *dev, uint32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_BATT_CYCLES, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((uint32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}

int jk_bms_read_charge_discharge_stat(jk_device_t *dev, uint8_t *charge, uint8_t *discharge)
{
	int res;
	uint8_t _dst[2];
	res = modbus_rtu_read_reg(&dev->modbus, JK_MODBUS_REG_CHARGE_DISCHARGE_STAT, _dst);
	if(res == MODBUS_RTU_OK){
		*charge = _dst[0];
		*discharge = _dst[1];
	}

	return res;
}

int jk_bms_read_alarms(jk_device_t *dev, uint32_t *dst)
{
	int res;
	uint8_t _dst[4];
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_ALARMS, 2, _dst);
	if(res == MODBUS_RTU_OK){
		*dst = ((uint32_t)_dst[0]) << 24;
		*dst |= ((uint32_t)_dst[1]) << 16;
		*dst |= ((uint32_t)_dst[2]) << 8;
		*dst |= _dst[3];
	}

	return res;
}

int jk_bms_toggle_charge(jk_device_t *dev, uint8_t state)
{
	uint8_t new_state = state == JK_MOSFET_ENABLE ? JK_MOSFET_ENABLE: JK_MOSFET_DISABLE;
	const uint8_t data[] = {0, 0, 0, new_state};
	return modbus_rtu_write_register(&dev->modbus, JK_MODBUS_REG_TOGGLE_CHARGE, 2, data);
}

int jk_bms_toggle_discharge(jk_device_t *dev, uint8_t state)
{
	uint8_t new_state = state == JK_MOSFET_ENABLE ? JK_MOSFET_ENABLE: JK_MOSFET_DISABLE;
	const uint8_t data[] = {0, 0, 0, new_state};
	return modbus_rtu_write_register(&dev->modbus, JK_MODBUS_REG_TOGGLE_DISCHARGE, 2, data);
}

int jk_bms_toggle_rs485(jk_device_t *dev, uint8_t state)
{
	const uint8_t data[] = {0, state << 3};
	return modbus_rtu_write_register(&dev->modbus, 0x1114, 1, data);
}

int jk_bms_read_cell_voltages(jk_device_t *dev, uint16_t *jk_cells)
{
	int res;
	uint8_t _dst[40];
	uint8_t j = 0;
	res = modbus_rtu_read_reg_mutiple(&dev->modbus, JK_MODBUS_REG_CELL_VOLTAGE, 40/2, _dst);
	if(res == MODBUS_RTU_OK){
		for (uint8_t i = 0; i < 40/2; i++)
		{
			jk_cells[i] = ((uint16_t)_dst[j]) << 8;
			jk_cells[i] |= _dst[j+1];
			j += 2;
		}
	}

	return res;
}

int jk_bms_change_device_address(jk_device_t *dev, uint8_t dev_address)
{
	const uint8_t data[] = {0, 0, 0, dev_address};
	uint8_t write_cmd[128];
	uint8_t write_rsp[16];
	int write_count;
	int read_count;
	uint16_t crc;
	uint16_t rcv_crc;

	write_count = modbus_rtu_create_write_packet(dev->modbus.device_addr, JK_MODBUS_REG_DEVICE_ADDRESS, 2, 2*2, data, write_cmd, sizeof(write_cmd));
	if(write_count <= 0){
		return -MODBUS_RTU_ERR;
	}

	if(dev->modbus.interface.write(write_cmd, write_count) < 0){
		return -MODBUS_RTU_ERR;
	}

	read_count = dev->modbus.interface.read(write_rsp, sizeof(write_rsp));
	if(read_count <= 0){
		return -MODBUS_RTU_ERR_TIMEOUT;
	}

	if(write_rsp[0] != data[3] || write_rsp[1] != MODBUS_RTU_WRITE_CMD){ // response has the slave address as the new one
		return -MODBUS_RTU_ERR;
	}

	crc = modbus_rtu_calculate_crc(write_rsp, read_count - 2);

	rcv_crc = write_rsp[read_count-1];
	rcv_crc <<= 8;
	rcv_crc |= write_rsp[read_count-2];

	if(rcv_crc != crc){
		return MODBUS_RTU_ERR_CRC;
	}

	return MODBUS_RTU_OK;
}

int jk_bms_read_serial(jk_device_t *dev, char *serial_number)
{
	const uint8_t data[] = {0, 0};
	uint8_t dst[512] = {0};
	int write_count;
	uint8_t write_cmd[128];
	int read_count;

	write_count = modbus_rtu_create_write_packet(dev->modbus.device_addr, 0x161C, 1, 1*2, data, write_cmd, sizeof(write_cmd));
	if(write_count <= 0){
		return -MODBUS_RTU_ERR;
	}

	if(dev->modbus.interface.write(write_cmd, write_count) < 0){
		return -MODBUS_RTU_ERR;
	}

	read_count = dev->modbus.interface.read(dst, sizeof(dst));
	if(read_count <= 0){
		return -MODBUS_RTU_ERR_TIMEOUT;
	}

	if(dst[0] != 0x55 && dst[1] != 0xAA && dst[2] != 0xEB && dst[3] != 0x90 && dst[4] != 0x03){
		return -MODBUS_RTU_ERR;
	}
	
	//memcpy(serial_number, &(dst[86]), 12); // no crc checking here
	memcpy(serial_number, &(dst[86]), 16); // changed from reading 12 bytes from index 86 to 16 bytes

	return MODBUS_RTU_OK;
}

int jk_bms_read_all(jk_device_t *dev)
{
	uint8_t data[] = {0x00, 0x00};
	return modbus_rtu_write_register(&dev->modbus, 0x1620, 1, data);
}
