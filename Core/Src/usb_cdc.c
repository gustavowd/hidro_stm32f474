/*
 * usb_cdc.c
 *
 *  Created on: 19 de jun. de 2026
 *      Author: gustavo
 */

#include <stdbool.h>
#include "main.h"
#include "cmsis_os.h"
#include "tusb.h"

SemaphoreHandle_t cdc_tx_sem;
QueueHandle_t	  cdc_rx_queue;

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_device_task(void* param)
{
  (void) param;

  // init device stack on configured roothub port
  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tud_init(BOARD_TUD_RHPORT);

  // RTOS forever loop
  while (1)
  {
    // put this thread to waiting state until there is new events
    tud_task();

    // following code only run if tud_task() process at least 1 event
    tud_cdc_write_flush();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  //xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  //xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  //xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  //xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}


SemaphoreHandle_t cdc_tx_sem;
QueueHandle_t	  cdc_rx_queue;
QueueHandle_t 	  queue_button;

#define USB_PACKET_SIZE	64

void tud_cdc_send(uint8_t *buffer, uint32_t bufsize, TickType_t timeout){
	if (bufsize <= USB_PACKET_SIZE){
		tud_cdc_write((uint8_t *)buffer, bufsize);
	    tud_cdc_write_flush();
	    xSemaphoreTake(cdc_tx_sem, timeout);
	}else{
		uint32_t len = 0;
		while(bufsize){
			if (bufsize > USB_PACKET_SIZE){
				len = USB_PACKET_SIZE;
			}else{
				len = bufsize;
			}
			tud_cdc_write((uint8_t *)buffer, len);
			tud_cdc_write_flush();
			xSemaphoreTake(cdc_tx_sem, timeout);
			buffer += len;
			bufsize -= len;
		}
	}
}

uint32_t tud_cdc_receive(uint8_t *buffer, uint32_t bufsize, TickType_t timeout){
	uint32_t len;
	xQueueReceive(cdc_rx_queue, &len, timeout);
	if (len > bufsize){
		len = bufsize;
	}
	uint32_t count = tud_cdc_read(buffer, len);
	return count;
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void* params)
{
  (void) params;
  uint8_t buffer[16];
  cdc_rx_queue = xQueueCreate(8, sizeof(uint32_t));
  cdc_tx_sem = xSemaphoreCreateBinary();

	do {
		vTaskDelay(10);
	}while (!tud_cdc_connected());

  // RTOS forever loop
  while ( 1 )
  {
		/* This implementation reads a single character at a time.  Wait in the
		Blocked state until a character is received. */
		// read
		uint32_t count = tud_cdc_receive(buffer, 1, portMAX_DELAY);

        // read and echo back
		if (count){
			tud_cdc_send(buffer, count, portMAX_DELAY);
		}
  }
}


// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;

  // TODO set some indicator
  if ( dtr )
  {
    // Terminal connected
  }else
  {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	portBASE_TYPE high_priority_task_woken = pdFALSE;
	uint32_t len = tud_cdc_n_available(itf);
	xQueueSendToBackFromISR(cdc_rx_queue, &len, &high_priority_task_woken);
	portYIELD_FROM_ISR(high_priority_task_woken)
}


void tud_cdc_tx_complete_cb(uint8_t itf) {
	  (void) itf;
	  xSemaphoreGive(cdc_tx_sem);
}
