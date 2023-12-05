// UART0 driver
//
// Copyright (C) 2023 Prusa Research a.s - www.prusa3d.com
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

#define MAX_PACKET_SIZE 2048

#ifdef CONFIG_IDF_TARGET_ESP8266
extern int critical_mutex;
#define custom_taskENTER_CRITICAL(m) taskENTER_CRITICAL()
#define custom_taskEXIT_CRITICAL(m) taskEXIT_CRITICAL()
#endif
#ifdef CONFIG_IDF_TARGET_ESP32
extern portMUX_TYPE critical_mutex;
#define custom_taskENTER_CRITICAL(m) taskENTER_CRITICAL(m)
#define custom_taskEXIT_CRITICAL(m) taskEXIT_CRITICAL(m)
#endif

// Register custom UART0 driver.
// For description of `uart0_tx_task_handle` parameters, see documentation of `uart0_tx_bytes()` function.
esp_err_t uart0_driver_install(TaskHandle_t uart0_tx_task_handle);

// Transmit `size` bytes from `data` into UART0.
// This must be called exclusively by RTOS task identified by `uart0_tx_task_handle`,
// because we are using task notifications to implement lightweight binary semaphore.
// Before calling this function for the first time, the task must call `ulTaskNotifyTake()`
// to ensure proper synchronization with `uart0_driver_install()`
void uart0_tx_bytes(uint8_t *data, size_t size);

// Let the driver know new message intron
void uart0_set_intron(uint8_t intron[8]);

struct __attribute__((packed)) Header {
    uint8_t type;
    union {
        uint8_t version; // when type == MSG_DEVINFO_V2
        uint8_t unused;  // when type == MSG_CLIENTCONFIG_V2
        uint8_t up;      // when type == MSG_PACKET_V2
    };
    uint16_t size;
};

/**
 * @brief Buffer for receiving packets from UART0
 *
 * To avoid delays in reading UART FIFO this driver holds a queue of preallocated buffers. These are automatically used to receive
 * incoming messages. Due to this the driver is in charge of looking for intron and reading header in order to determine message size.
 * Beware, this is not neutral UART driver. It already understands basic protocol and handles it.
 * The user is supposed to obtain filled buffers with uart0_rx_get_buffer() and return them back with uart0_rx_release_buffer() once
 * processed by the application.
 */
typedef struct __attribute__((packed)) {
    struct Header header;
    uint8_t data[MAX_PACKET_SIZE];
} RxBuffer;

/// Get buffer containing packet from the queue
/// @return pointer to buffer to be filled with packet data
RxBuffer* uart0_rx_get_buffer();

/// Return processed buffer back to driver
/// @param buffer pointer to buffer to be returned
void uart0_rx_release_buffer(RxBuffer *buffer);
