#include "can_storage.h"
#include "uart_logger.h"

#include <string.h>

static can_storage_t storage = {
    .mutex = NULL,
    .nodes = {
        { .key = 0x2000004 },
        { .key = 0x2000104 },
        { .key = 0x2000204 },
        { .key = 0x2000304 },
        { .key = 0x2000404 },
        { .key = 0x4000504 },
        { .key = 0x4000604 },
        { .key = 0x4000704 },
        { .key = 0x4000804 },
        { .key = 0xA006404 },
        { .key = 0xA006504 },
        { .key = 0xA006604 },
        { .key = 0xA006704 },
        { .key = 0xA00C804 },
        { .key = 0xA00C904 },
        { .key = 0xA00CA04 },
        { .key = 0xA00CB04 },
        { .key = 0xA012C04 },
        { .key = 0xA012D04 },
        { .key = 0xA012E04 },
        { .key = 0xA012F04 },
        { .key = 0xA019004 },
        { .key = 0xA019104 },
        { .key = 0xA019204 },
        { .key = 0xA019304 },
        { .key = 0x8000008 },
        { .key = 0x4000108 },
        { .key = 0x208 },
        { .key = 0x308 },
        { .key = 0x408 },
        { .key = 0x508 },
        { .key = 0x6000110 },
        { .key = 0x6000210 },
        { .key = 0x6000310 },
        { .key = 0x6000410 },
        { .key = 0x6000510 },
        { .key = 0x6000610 },
        { .key = 0x6000710 },
        { .key = 0x6000810 },
        { .key = 0x6000910 },
        { .key = 0x6000A10 },
        { .key = 0x400010C },
        { .key = 0x800020C },
        { .key = 0x600050C },
        { .key = 0x600060C },
        { .key = 0x600070C },
        { .key = 0x600080C },
        { .key = 0x600090C },
        { .key = 0x6000A0C },
        { .key = 0xC000B0C },
        { .key = 0xC00640C },
        { .key = 0xC00650C },
        { .key = 0xC00660C },
        { .key = 0xC00670C },
        { .key = 0xC00680C },
        { .key = 0xC00690C },
        { .key = 0xC006A0C },
        { .key = 0xC006B0C },
        { .key = 0xC006C0C },
        { .key = 0xC006D0C },
        { .key = 0xC006E0C },
        { .key = 0xC006F0C },
        { .key = 0xC00700C },
        { .key = 0xC00710C },
        { .key = 0xC00720C },
        { .key = 0xC00730C },
        { .key = 0xC00740C },
        { .key = 0xC00750C },
        { .key = 0xC00760C },
        { .key = 0xC00770C },
        { .key = 0xC00780C },
        { .key = 0xC00790C },
        { .key = 0xC007A0C },
        { .key = 0xC007B0C },
		{ .key = 0x610     },
		{ .key = 0x611     },
		{ .key = 0x613     },
		{ .key = 0x614     },
		{ .key = 0x615     },
		{ .key = 0x620	   },
		{ .key = 0x621	   },
    }
};
void init_can_storage() {
	if (storage.mutex == NULL) {
		storage.mutex = osMutexNew(NULL);

		if (storage.mutex == NULL) {
			uart_logger_add_msg("[C_STOR] Failed to create mutex for CAN storage\r\n", 0);
		} else {
			uart_logger_add_msg("[C_STOR] Created mutex for CAN storage\r\n", 0);
		}
	}
}

void insert_can_msg_to_storage(const uint32_t key, can_payload_t* value) {
	// Find the index of the node with the given key
	int index = -1;

	for (int i = 0; i < TABLE_SIZE; i++) {
		if (storage.nodes[i].key == key) {
			index = i;
			break;
		}
	}

	if (index == -1) {
		uart_logger_add_msg("[C_STOR] Key not found in storage\r\n", 0);
		return;
	}

	// Acquire the mutex before modifying the storage
	if (osMutexAcquire(storage.mutex, osWaitForever) == osOK) {
		// Update the value and mark the node as valid
		storage.nodes[index].value.len = value->len;
		memcpy(storage.nodes[index].value.payload, value->payload, value->len);
		storage.nodes[index].valid = true;

		// Release the mutex after modification
		osMutexRelease(storage.mutex);
	} else {
		uart_logger_add_msg("[C_STOR] Failed to acquire mutex for CAN storage\r\n", 0);
	}
}

can_storage_t* get_can_storage() {
	return &storage;
}
