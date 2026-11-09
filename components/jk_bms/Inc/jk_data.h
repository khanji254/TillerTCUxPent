#ifndef INC_JK_DATA_H_
#define INC_JK_DATA_H_

#include <stdint.h>

typedef struct {
	union {
		uint16_t cells[20];
		struct{
			uint16_t cell0;
			uint16_t cell1;
			uint16_t cell2;
			uint16_t cell3;
			uint16_t cell4;
			uint16_t cell5;
			uint16_t cell6;
			uint16_t cell7;
			uint16_t cell8;
			uint16_t cell9;
			uint16_t cell10;
			uint16_t cell11;
			uint16_t cell12;
			uint16_t cell13;
			uint16_t cell14;
			uint16_t cell15;
			uint16_t cell16;
			uint16_t cell17;
			uint16_t cell18;
			uint16_t cell19;
		}cells_struct;
	}jk_cells;
}jk_cells_t;

typedef struct {
	uint32_t device_address;
	uint32_t batt_v;
	int32_t batt_i;
	uint32_t batt_power;
	int32_t remaining_capacity;
	uint32_t cell_count;
	uint32_t cells_present;
	uint16_t cells_diff;
	uint8_t soc;
	uint8_t soh;
	uint8_t charge_stat;
	uint8_t discharge_stat;
	uint32_t charge_cycles;
	uint32_t alarms;
	char model_no[32];
	char serial_no[32];
	jk_cells_t jk_cells;
}jk_data_t;

#endif /* INC_JK_DATA_H_ */