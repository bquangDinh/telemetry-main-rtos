#ifndef INC_CAN_STORAGE_H_
#define INC_CAN_STORAGE_H_

#include "cmsis_os.h"
#include <stdbool.h>

#define TABLE_SIZE 86
#define MAX_VALUE_LENGTH 64

typedef struct can_payload {
	uint8_t len;
	uint8_t payload[MAX_VALUE_LENGTH];
} can_payload_t;

typedef struct can_msg_node {
	uint32_t key;
	can_payload_t value;
	bool valid;
} can_msg_node_t;

typedef struct can_storage {
	can_msg_node_t nodes[TABLE_SIZE];
	osMutexId_t mutex;
} can_storage_t;

void CAN_STORAGE_Task_Init();

void insert_can_msg_to_storage(const uint32_t key, can_payload_t* value);

can_storage_t* get_can_storage();
#endif /* INC_CAN_DATA_STORAGE_H_ */