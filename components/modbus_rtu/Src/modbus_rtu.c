/*
 * modbus_rtu.c
 *
 *  Created on: Jun 19, 2025
 *      Author: collo
 *      Addons: don
 */


#include "modbus_rtu.h"

uint16_t modbus_rtu_calculate_crc(const unsigned char *data, const uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];          // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
        if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
        }
        else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}

int modbus_rtu_create_read_packet(const unsigned char addr, const uint16_t start_reg, const uint16_t reg_num, unsigned char *dst, const uint16_t dst_len)
{
	int index = 0;
	uint16_t crc;
	
	if(dst_len < 8){
		return -1;
	}

	dst[index++] = addr;
	dst[index++] = 0x03;
	dst[index++] = (start_reg >> 8) & 0xff;
	dst[index++] = start_reg & 0xff;
	dst[index++] = (reg_num >> 8) & 0xff;
	dst[index++] = reg_num & 0xff;
	crc = modbus_rtu_calculate_crc(dst, index);
	dst[index++] = crc & 0xff;
	dst[index++] = (crc >> 8) & 0xff;

	return index;
}

int modbus_rtu_create_write_packet(const unsigned char addr, const uint16_t start_reg, const uint16_t reg_num, const uint16_t byte_count, const unsigned char *data, unsigned char *dst, const uint16_t dst_len)
{
	int index = 0;
	uint16_t crc;

	if(dst_len < (byte_count + 7)){
		return -1;
	}

	dst[index++] = addr;
	dst[index++] = 0x10;
	dst[index++] = (start_reg >> 8) & 0xff;
	dst[index++] = start_reg & 0xff;
	dst[index++] = (reg_num >> 8) & 0xff;
	dst[index++] = reg_num & 0xff;
	dst[index++] = byte_count;

	for(int i=0; i<byte_count; ++i){
		dst[index++] = data[i];
	}

	crc = modbus_rtu_calculate_crc(dst, index);
	dst[index++] = crc & 0xff;
	dst[index++] = (crc >> 8) & 0xff;

	return index;
}

int modbus_rtu_read_reg(modbus_rtu_dev_t *dev, uint16_t reg_addr, uint8_t *dst)
{
	uint8_t read_cmd[16]={0};
	uint8_t rsp[32]={0};
	int write_count;
	int rcv_count;
	uint16_t crc;
	uint16_t rcv_crc;

	write_count = modbus_rtu_create_read_packet(dev->device_addr, reg_addr, 1, read_cmd, sizeof(read_cmd));
	if(write_count <= 0){
		return -MODBUS_RTU_ERR;
	}

	if(dev->interface.write(read_cmd, write_count) < 0){
		return -MODBUS_RTU_ERR;
	}

	rcv_count = dev->interface.read(rsp, sizeof(rsp));
	if(rcv_count <= 0){
		return -MODBUS_RTU_ERR_TIMEOUT;
	}

	if(rsp[0] != dev->device_addr || rsp[1] != MODBUS_RTU_READ_CMD || rsp[2] != 0x02){
		return -MODBUS_RTU_ERR;
	}

	crc = modbus_rtu_calculate_crc(rsp, rcv_count - 2);

	rcv_crc = rsp[rcv_count-1];
	rcv_crc <<= 8;
	rcv_crc |= rsp[rcv_count-2];

	if(rcv_crc != crc){
		return -MODBUS_RTU_ERR_CRC;
	}

	dst[0] = rsp[3];
	dst[1] = rsp[4];

	return MODBUS_RTU_OK;
}

int modbus_rtu_read_reg_mutiple(modbus_rtu_dev_t *dev, uint16_t reg_addr, uint16_t reg_count, uint8_t *dst)
{
	uint8_t read_cmd[16]={0};
	uint8_t rsp[128]={0};
	int write_count;
	int rcv_count;
	uint16_t crc;
	uint16_t rcv_crc;

	write_count = modbus_rtu_create_read_packet(dev->device_addr, reg_addr, reg_count, read_cmd, sizeof(read_cmd));
	if(write_count <= 0){
		return -MODBUS_RTU_ERR;
	}

	if(dev->interface.write(read_cmd, write_count) < 0){
		return -MODBUS_RTU_ERR;
	}

	rcv_count = dev->interface.read(rsp, sizeof(rsp));
	if(rcv_count <= 0){
		return -MODBUS_RTU_ERR_TIMEOUT;
	}

	if(rsp[0] != dev->device_addr || rsp[1] != MODBUS_RTU_READ_CMD || rsp[2] != (reg_count * 2)){
		return -MODBUS_RTU_ERR;
	}

	crc = modbus_rtu_calculate_crc(rsp, rcv_count - 2);

	rcv_crc = rsp[rcv_count-1];
	rcv_crc <<= 8;
	rcv_crc |= rsp[rcv_count-2];

	if(rcv_crc != crc){
		return MODBUS_RTU_ERR_CRC;
	}

	for(int i=0; i<reg_count*2; ++i){
		dst[i] = rsp[i+3];
	}

	return MODBUS_RTU_OK;
}

int modbus_rtu_write_register(modbus_rtu_dev_t *dev, uint16_t reg_addr, uint16_t reg_num, const uint8_t *data)
{
	uint8_t write_cmd[128];
	uint8_t write_rsp[16];
	int write_count;
	int read_count;
	uint16_t crc;
	uint16_t rcv_crc;

	write_count = modbus_rtu_create_write_packet(dev->device_addr, reg_addr, reg_num, reg_num*2, data, write_cmd, sizeof(write_cmd));
	if(write_count <= 0){
		return -MODBUS_RTU_ERR;
	}

	if(dev->interface.write(write_cmd, write_count) < 0){
		return -MODBUS_RTU_ERR;
	}

	read_count = dev->interface.read(write_rsp, sizeof(write_rsp));
	if(read_count <= 0){
		return -MODBUS_RTU_ERR_TIMEOUT;
	}

	if(write_rsp[0] != dev->device_addr || write_rsp[1] != MODBUS_RTU_WRITE_CMD){
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