/*
 * sd_card.c
 *
 *  Created on: Mar 29, 2026
 *      Author: buiqu
 */

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "sd_card.h"
#include "uart_logger.h"
#include "cmsis_os.h"
#include "fatfs.h"
//#include "diskio.h"

#define SDCARD_DETECT_LED_PIN SD_DETECT_LED_Pin
#define SDCARD_DETECT_LED_PORT SD_DETECT_LED_GPIO_Port

#define SDCARD_RW_LED_PIN RW_LED_Pin
#define SDCARD_RW_LED_PORT RW_LED_GPIO_Port

// Sync every 500ms
#define SYNC_INTERVAL 500

#define DATA_FILE "data.dat"
#define TEST_FILE "test.txt"

typedef struct {
	uint32_t time;
	uint8_t len;
	uint8_t payload[SDCARD_MAX_DATA_LEN];
} sdcard_payload_t;

typedef struct {
	sdcard_payload_t buffer[SDCARD_PAYLOAD_QUEUE_MAX_CAPACITY];
	sdcard_payload_t pending_payload;
	bool payload_processing;
	uint8_t retries_count;
	uint16_t tail;
	uint16_t head;
	uint16_t count;
	osMutexId_t mutex;
	osSemaphoreId_t items;
} sdcard_queue_t;

static void SDCARD_Task(void *argument);

static osThreadId_t sdcardTaskHandler;

static SD_HandleTypeDef *sdcard = NULL;

static uint8_t current_retry_count = 0;

static sdcard_queue_t sdcard_payload_queue;

static uint32_t last_sync = 0;

static FATFS fs;

static FIL file;

static const osThreadAttr_t sdcardTaskAttr = { .name = "sdcardTask",
		.stack_size = 4096 * 4, .priority = (osPriority_t) osPriorityNormal };

static SDCardState_t sdcard_state = SDCARD_STATE_RESET;

volatile sdcard_health_state_t sdcard_health_state = { .last_progress = 0,
		.wait_start = 0, .current_state = SDCARD_STATE_RESET };

/**
 * SD Card state handlers
 */
static bool sdcard_handle_reset();
static bool sdcard_handle_mount_open();
static bool sdcard_handle_ready();
static bool sdcard_handle_error();
static bool sdcard_handle_disabled();

/**
 * SD Card Queue implementation
 */
static bool sdcard_payload_pop(sdcard_payload_t *out);
static void reset_queue();

static bool write_to_card(const sdcard_payload_t *payload);

/**
 * SD Card check. Those are used to make sure the SD card is fully functional
 */
static bool perform_sd_check();
static bool check_card_status();
static bool check_sd_raw_read();
static bool check_sd_mount();
static bool check_sd_write_sync();
static bool check_sd_open_create();
static bool sdcard_mount_fs(void);
static bool sdcard_unmount_fs(void);
static bool sdcard_open_file(FIL *target, const char *path, BYTE mode);
static bool sdcard_close_file(FIL *target);
static bool sdcard_test_file_access(const char *open_failure_msg,
		bool write_data);

/**
 * Other related functioons
 */
static void blink_rw_led();

/**
 * Logging functions
 */
#if SD_LOG_ENABLED

// real implementations
static void sd_logln_impl(const char *msg);
static void sd_log_fmt_ln_impl(const char *fmt, ...);
static void sd_log_init_error_impl(uint32_t err);
static void sd_log_fs_error_impl(FRESULT err);

// macros map to real functions
#define sd_logln(msg)              sd_logln_impl(msg)
#define sd_log_fmt_ln(fmt, ...)    sd_log_fmt_ln_impl(fmt, ##__VA_ARGS__)
#define sd_log_init_error(err)     sd_log_init_error_impl(err)
#define sd_log_fs_error(err)       sd_log_fs_error_impl(err)

#else

// removed at compile time
#define sd_logln(msg)              ((void)0)
#define sd_log_raw(msg)            ((void)0)
#define sd_log_fmt_ln(fmt, ...)    ((void)0)
#define sd_log_init_error(err)     ((void)0)
#define sd_log_fs_error(err)       ((void)0)

#endif

void SDCARD_Task_Init(SD_HandleTypeDef *_sdcard_instace) {
	sdcard = _sdcard_instace;

	sdcard_payload_queue.mutex = osMutexNew(NULL);

	sdcard_payload_queue.items = osSemaphoreNew(
	SDCARD_PAYLOAD_QUEUE_MAX_CAPACITY, 0, NULL);

	sdcardTaskHandler = osThreadNew(SDCARD_Task, NULL, &sdcardTaskAttr);

	if (sdcardTaskHandler == NULL) {
		sd_logln("Failed to create task");
	}
}

bool SDCARD_add_payload_to_queue(const char *message, const uint16_t len) {
	if (message == NULL || len == 0)
		return false;

	if (len >= SDCARD_MAX_DATA_LEN) {
		// Ignore oversize message
		return false;
	}

	osMutexAcquire(sdcard_payload_queue.mutex, osWaitForever);

	if (sdcard_payload_queue.count >= SDCARD_PAYLOAD_QUEUE_MAX_CAPACITY) {
		osMutexRelease(sdcard_payload_queue.mutex);

		return false;
	}

	sdcard_payload_t *slot =
			&sdcard_payload_queue.buffer[sdcard_payload_queue.head];

	slot->len = len;

	memcpy(slot->payload, message, len);

	slot->time = osKernelGetTickCount();

	sdcard_payload_queue.head = (sdcard_payload_queue.head + 1)
			% SDCARD_PAYLOAD_QUEUE_MAX_CAPACITY;

	sdcard_payload_queue.count++;

	osMutexRelease(sdcard_payload_queue.mutex);

	// Signal the task there is item to consume
	osSemaphoreRelease(sdcard_payload_queue.items);

	return true;
}

static void SDCARD_Task(void *argument) {
	sd_logln("Initializing sd card...");

	sdcard_health_state.last_progress = osKernelGetTickCount();

	while (1) {
		sdcard_health_state.wait_start = osKernelGetTickCount();

		switch (sdcard_state) {
		case SDCARD_STATE_RESET:
			if (sdcard_handle_reset()) {
				sdcard_state = SDCARD_STATE_MOUNT_AND_OPEN;
			} else {
				sdcard_state = SDCARD_STATE_ERROR;
			}
			break;
		case SDCARD_STATE_MOUNT_AND_OPEN:
			if (sdcard_handle_mount_open()) {
				sdcard_state = SDCARD_STATE_READY;
			} else {
				sdcard_state = SDCARD_STATE_ERROR;
			}
			break;
		case SDCARD_STATE_READY:
			if (!sdcard_handle_ready()) {
				sdcard_state = SDCARD_STATE_ERROR;
			}
			break;
		case SDCARD_STATE_ERROR:
			sdcard_handle_error();
			break;
		case SDCARD_STATE_DISABLED:
			sdcard_handle_disabled();
			break;
		}

		sdcard_health_state.last_progress = osKernelGetTickCount();
		sdcard_health_state.current_state = sdcard_state;

		osDelay(100);
	}
}

static bool sdcard_handle_reset() {
	sd_logln("Resetting SD Card...");

	// Make sure the RW LED is OFF
	HAL_GPIO_WritePin(SDCARD_RW_LED_PORT, SDCARD_RW_LED_PIN, GPIO_PIN_RESET);

	reset_queue();

	// Deinit the SD card since we would try to reset maybe more than one time
	// we don't want the previous state of sdcard to be around
	HAL_SD_DeInit(sdcard);

	// Init SD card again
	HAL_StatusTypeDef ret = HAL_SD_Init(sdcard);

	if (ret != HAL_OK) {
#if SD_LOG_ENABLED
		uint32_t err = HAL_SD_GetError(sdcard);

		sd_log_init_error(err);

		sd_logln("Failed to init sd card. SD Card may not present");
#endif
		return false;
	}

	// Check if we can read its status and some information about the SD card
	if (!perform_sd_check()) {
		sd_logln("Card status check failed!");

		return false;
	} else {
		sd_logln("SD card fully functional!");
	}

	// Since we can read SD card, that means SD card is def present
	// Turn the LED on
	HAL_GPIO_WritePin(SDCARD_DETECT_LED_PORT, SDCARD_DETECT_LED_PIN,
			GPIO_PIN_SET);

	sd_logln("Reset SD Card successfully");

	return true;
}

static bool sdcard_handle_mount_open() {
//    // Check disk layer first
//    DSTATUS init_stat = disk_initialize(0);
//    DSTATUS cur_stat  = disk_status(0);
//
//    snprintf(msg, sizeof(msg),
//             "[SD] disk_init=0x%02X, disk_status=0x%02X\r\n",
//             init_stat, cur_stat);
//    uart_logger_add_msg(msg, strlen(msg));

	// Mount file system
	sd_logln("Mounting file system...");

	if (!sdcard_mount_fs()) {
		sd_logln("Failed to mount file system");

		return false;
	}

	sd_logln("Mounted fs. Open data file...");

	// Open or create file
	if (!sdcard_open_file(&file, DATA_FILE, FA_OPEN_APPEND | FA_WRITE)) {
		sd_log_fmt_ln("Failed to open data file");

		// Unmount
		sdcard_unmount_fs();

		return false;
	}

	sd_logln("Mount and open OK!");

	return true;
}

static bool sdcard_handle_ready() {
//	sdcard_payload_t test_payload = { .len = 14, .payload = "Hello World!!!",
//			.time = osKernelGetTickCount() };
//
//	if (write_to_card(&test_payload)) {
//		sd_logln("Wrote test data ok");
//	} else {
//		sd_logln("Failed to write data");
//	}

	bool payload_available = false;

	if (!sdcard_payload_queue.payload_processing) {
		if (osSemaphoreAcquire(sdcard_payload_queue.items, 1000) == osOK) {
			payload_available = sdcard_payload_pop(
					&sdcard_payload_queue.pending_payload);
		}
	}

	if (payload_available || sdcard_payload_queue.payload_processing) {
		if (sdcard_payload_queue.retries_count >= SDCARD_PAYLOAD_WRITE_RETRIES) {
			sdcard_payload_queue.payload_processing = false;

			sd_logln("Failed to write payload. Max retries reached");

			return false;
		}

		if (write_to_card(&sdcard_payload_queue.pending_payload)) {
			sd_logln("Wrote data ok");

			sdcard_payload_queue.payload_processing = false;

			sdcard_payload_queue.retries_count = 0;
		} else {
			sd_logln("Failed to write data");

			sdcard_payload_queue.retries_count++;

			sdcard_payload_queue.payload_processing = true;
		}
	}

	return true;
}

static bool sdcard_handle_error() {
	if (current_retry_count < SDCARD_NUM_RETRIES) {
		// Attempt to reset SD Card again
		sdcard_state = SDCARD_STATE_RESET;

		current_retry_count++;
	} else {
		sd_logln("Number of retries reached. Disabled SD Card");

		sdcard_state = SDCARD_STATE_DISABLED;
	}

	return true;
}

static bool sdcard_handle_disabled() {
	// Sleep this task forever
	// So the only way to re-enable SD Card again is to reset the board
	osThreadSuspend(osThreadGetId());

	return true;
}

static bool sdcard_payload_pop(sdcard_payload_t *out) {
	if (out == NULL)
		return false;

	osMutexAcquire(sdcard_payload_queue.mutex, osWaitForever);

	if (sdcard_payload_queue.count == 0) {
		osMutexRelease(sdcard_payload_queue.mutex);

		return false;
	}

	*out = sdcard_payload_queue.buffer[sdcard_payload_queue.tail];

	sdcard_payload_queue.tail = (sdcard_payload_queue.tail + 1)
			% SDCARD_PAYLOAD_QUEUE_MAX_CAPACITY;

	sdcard_payload_queue.count--;

	osMutexRelease(sdcard_payload_queue.mutex);

	return true;
}

static bool sdcard_mount_fs(void) {
	FRESULT res = f_mount(&fs, "", 1);

	if (res != FR_OK) {
		sd_log_fs_error(res);
		return false;
	}

	return true;
}

static bool sdcard_unmount_fs(void) {
	FRESULT res = f_mount(NULL, "", 1);

	if (res != FR_OK) {
		sd_log_fs_error(res);
		return false;
	}

	return true;
}

static bool sdcard_open_file(FIL *target, const char *path, BYTE mode) {
	FRESULT res = f_open(target, path, mode);

	if (res != FR_OK) {
		sd_log_fs_error(res);
		return false;
	}

	return true;
}

static bool sdcard_close_file(FIL *target) {
	FRESULT res = f_close(target);

	if (res != FR_OK) {
		sd_log_fs_error(res);
		return false;
	}

	return true;
}

static bool sdcard_test_file_access(const char *open_failure_msg,
		bool write_data) {
	FIL test_file;
	const char *msg = "OK\r\n";
	UINT byte_written = 0;

	if (!sdcard_mount_fs()) {
		return false;
	}

	if (!sdcard_open_file(&test_file, TEST_FILE, FA_OPEN_APPEND | FA_WRITE)) {
		sd_logln(open_failure_msg);
		sdcard_unmount_fs();
		return false;
	}

	if (write_data) {
		FRESULT res = f_write(&test_file, msg, strlen(msg), &byte_written);

		if (res != FR_OK || byte_written != strlen(msg)) {
			sd_logln("Write file not OK!");
			sd_log_fs_error(res);
			sdcard_close_file(&test_file);
			sdcard_unmount_fs();
			return false;
		}
	}

	if (!sdcard_close_file(&test_file)) {
		sd_logln("Close file not OK!");
		sdcard_unmount_fs();
		return false;
	}

	return sdcard_unmount_fs();
}

static void reset_queue() {
	osMutexAcquire(sdcard_payload_queue.mutex, osWaitForever);

	// Reset the queue
	sdcard_payload_queue.head = 0;
	sdcard_payload_queue.tail = 0;
	sdcard_payload_queue.retries_count = 0;
	sdcard_payload_queue.payload_processing = 0;

	// drain semaphore
	while (osSemaphoreAcquire(sdcard_payload_queue.items, 0) == osOK) {
	}

	osMutexRelease(sdcard_payload_queue.mutex);
}

static bool write_to_card(const sdcard_payload_t *payload) {
	assert_param(payload != NULL);

	char msg[256];
	FRESULT res;
	UINT byte_written;

	int len = snprintf(msg, sizeof(msg), "%lu %u %.*s\n", payload->time,
			payload->len, payload->len, payload->payload);

	if (len < 0 || len >= sizeof(msg)) {
		return false;
	}

	res = f_write(&file, msg, len, &byte_written);

	if (res != FR_OK || byte_written != (UINT) len) {
		return false;
	}

	uint32_t now = osKernelGetTickCount();

	bool should_sync = (now - last_sync) >= SYNC_INTERVAL;

	if (should_sync) {
		f_sync(&file);

		last_sync = now;
	}

	// Indicate RW is done
	blink_rw_led();

	return true;
}

static void blink_rw_led() {
	HAL_GPIO_WritePin(SDCARD_RW_LED_PORT, SDCARD_RW_LED_PIN, GPIO_PIN_SET);

	osDelay(100);

	HAL_GPIO_WritePin(SDCARD_RW_LED_PORT, SDCARD_RW_LED_PIN, GPIO_PIN_RESET);

	osDelay(100);

	HAL_GPIO_WritePin(SDCARD_RW_LED_PORT, SDCARD_RW_LED_PIN, GPIO_PIN_SET);
}

#if SD_LOG_ENABLED

static void sd_logln_impl(const char *msg) {
	uart_logger_add_msg_format("[SD] %s\r\n", msg);
}

static void sd_log_fmt_ln_impl(const char *fmt, ...) {
	char buf[64];
	va_list args;

	va_start(args, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len > 0 && len < sizeof(buf)) {
		uart_logger_add_msg_format("[SD] %s\r\n", buf);
	}
}

static void sd_log_init_error_impl(uint32_t err) {
	if (err & HAL_SD_ERROR_CMD_RSP_TIMEOUT)
		sd_logln("CMD response timeout");

	if (err & HAL_SD_ERROR_CMD_CRC_FAIL)
		sd_logln("[SD] CMD CRC fail");

	if (err & HAL_SD_ERROR_DATA_TIMEOUT)
		sd_logln("[SD] Data timeout");

	if (err & HAL_SD_ERROR_DATA_CRC_FAIL)
		sd_logln("[SD] Data CRC fail");

	if (err & HAL_SD_ERROR_RX_OVERRUN)
		sd_logln("[SD] RX overrun");

	if (err & HAL_SD_ERROR_TX_UNDERRUN)
		sd_logln("[SD] TX underrun");

	if (err & HAL_SD_ERROR_ADDR_OUT_OF_RANGE)
		sd_logln("[SD] Address out of range");

	if (err & HAL_SD_ERROR_UNSUPPORTED_FEATURE)
		sd_logln("[SD] Unsupported feature");
}

static void sd_log_fs_error_impl(FRESULT err) {
	char msg[128];

	const char *meaning = "Unknown";

	switch (err) {
	case FR_OK:
		meaning = "Succeeded";
		break;

	case FR_DISK_ERR:
		meaning = "Low-level disk I/O error (read/write failed)";
		break;

	case FR_INT_ERR:
		meaning = "Internal error (likely memory/stack corruption)";
		break;

	case FR_NOT_READY:
		meaning = "Drive not ready (SD not initialized or not present)";
		break;

	case FR_NO_FILE:
		meaning = "File not found";
		break;

	case FR_NO_PATH:
		meaning = "Path not found";
		break;

	case FR_INVALID_NAME:
		meaning = "Invalid file/path name";
		break;

	case FR_DENIED:
		meaning = "Access denied or directory full";
		break;

	case FR_EXIST:
		meaning = "File already exists";
		break;

	case FR_INVALID_OBJECT:
		meaning = "Invalid file/directory object";
		break;

	case FR_WRITE_PROTECTED:
		meaning = "Media is write protected";
		break;

	case FR_INVALID_DRIVE:
		meaning = "Invalid drive number";
		break;

	case FR_NOT_ENABLED:
		meaning = "Volume not mounted";
		break;

	case FR_NO_FILESYSTEM:
		meaning = "No valid FAT filesystem (maybe exFAT or unformatted)";
		break;

	case FR_TIMEOUT:
		meaning = "Timeout waiting for resource";
		break;

	case FR_LOCKED:
		meaning = "File locked";
		break;

	case FR_NOT_ENOUGH_CORE:
		meaning = "Not enough memory";
		break;

	case FR_TOO_MANY_OPEN_FILES:
		meaning = "Too many open files";
		break;

	case FR_INVALID_PARAMETER:
		meaning = "Invalid parameter";
		break;

	case FR_MKFS_ABORTED:
		meaning = "MKFS aborted";
		break;
	}

	int len = snprintf(msg, sizeof(msg), "FS error %d: %s", err, meaning);

	if (len > 0 && len < sizeof(msg)) {
		sd_logln(msg);
	}
}

#endif

static bool perform_sd_check() {
	return check_card_status() && check_sd_raw_read() && check_sd_mount()
			&& check_sd_write_sync() && check_sd_open_create();
}

static bool check_card_status() {
	sd_logln("Checking SD card status...");

	HAL_SD_CardInfoTypeDef card_info;

	if (HAL_SD_GetCardInfo(sdcard, &card_info) != HAL_OK) {
		sd_logln("Failed to obtain sd card info");

		return false;
	}

	sd_log_fmt_ln("[SD] blk sz: %lu | typ: %lu | ver: %lu | cls: %lu",
			card_info.BlockSize, card_info.CardType, card_info.CardVersion,
			card_info.Class);

	return true;
}

static bool check_sd_raw_read() {
	sd_logln("Checking sd raw read...");

	static uint8_t block[512] __attribute__((aligned(32)));

	HAL_StatusTypeDef res = HAL_SD_ReadBlocks(sdcard, block, 0, 1, 1000);

	if (res != HAL_OK) {
		sd_log_fs_error(res);

		return false;
	}

	while (HAL_SD_GetCardState(sdcard) != HAL_SD_CARD_TRANSFER)
		osDelay(1);

	return true;
}

static bool check_sd_mount() {
	sd_logln("Checking sd mount...");

	if (!sdcard_mount_fs()) {
		sd_logln("Mount not OK!");

		return false;
	}

	// Unmount
	if (!sdcard_unmount_fs()) {
		sd_logln("Unmount not OK!");

		return false;
	}

	return true;
}

static bool check_sd_write_sync() {
	sd_logln("Checking sd write sync...");
	return sdcard_test_file_access("File open not OK!", true);
}

static bool check_sd_open_create() {
	sd_logln("Checking sd open and create...");
	return sdcard_test_file_access("Open file not OK!", false);
}
