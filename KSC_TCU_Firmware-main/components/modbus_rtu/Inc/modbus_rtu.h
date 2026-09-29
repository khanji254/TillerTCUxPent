/*
 * modbus.h
 *
 *  Created on: Jun 19, 2025
 *      Author: collo
 *      Addons: don
 */

#ifndef INC_MODBUS_RTU_H_
#define INC_MODBUS_RTU_H_

#define MODBUS_RTU_READ_CMD 0X03
#define MODBUS_RTU_WRITE_CMD 0X10

#include <stdint.h>

enum MODBUS_RTU_COMM_ENUM{
	MODBUS_RTU_OK,
	MODBUS_RTU_ERR,
	MODBUS_RTU_ERR_CRC,
	MODBUS_RTU_ERR_TIMEOUT,
};

typedef int (*modbus_rtu_interface_write)(const uint8_t *data, const uint16_t data_len);
typedef int (*modbus_rtu_interface_read)(uint8_t *dst, uint16_t len);

struct modbus_rtu_interface_s{
	modbus_rtu_interface_write write;
	modbus_rtu_interface_read read;
};

typedef struct{
	struct modbus_rtu_interface_s interface;
	uint8_t device_addr;
}modbus_rtu_dev_t;


uint16_t modbus_rtu_calculate_crc(const unsigned char *data, const uint16_t len);
int modbus_rtu_create_read_packet(const unsigned char addr, const uint16_t start_reg, const uint16_t reg_num, unsigned char *dst, const uint16_t dst_len);
int modbus_rtu_create_write_packet(const unsigned char addr, const uint16_t start_reg, const uint16_t reg_num, const uint16_t byte_count, const unsigned char *data, unsigned char *dst, const uint16_t dst_len);
int modbus_rtu_read_reg(modbus_rtu_dev_t *dev, uint16_t reg_addr, uint8_t *dst);
int modbus_rtu_read_reg_mutiple(modbus_rtu_dev_t *dev, uint16_t reg_addr, uint16_t reg_count, uint8_t *dst);
int modbus_rtu_write_register(modbus_rtu_dev_t *dev, uint16_t reg_addr, uint16_t reg_num, const uint8_t *data);

#endif /* INC_MODBUS_RTU_H_ */
