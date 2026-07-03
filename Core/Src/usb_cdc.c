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

#include "pb_encode.h"
#include "cobs.h"
#include "dados.pb.h"
#include "crc32.h"

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

// Tamanho do Protobuf (calculado automaticamente pela Nanopb)
#define BUFFER_PROTOBUF_SIZE   MensagemSerial_size + 4

// Tamanho do COBS aplicando a fórmula do pior cenário + 1 byte do 0x00 final
#define BUFFER_COBS_SIZE       (BUFFER_PROTOBUF_SIZE + (BUFFER_PROTOBUF_SIZE / 254))

uint8_t buffer_protobuf[BUFFER_PROTOBUF_SIZE];
uint8_t buffer_cobs[BUFFER_COBS_SIZE];
void enviar_dados_massivos(uint32_t idx) {
	MensagemSerial envelope = MensagemSerial_init_zero;

	envelope.which_conteudo = MensagemSerial_bloco_dados_tag;

	envelope.conteudo.bloco_dados.timestamp = xTaskGetTickCount();
	envelope.conteudo.bloco_dados.id_bloco = idx;
	envelope.conteudo.bloco_dados.leituras[0] = 1.0f;
	envelope.conteudo.bloco_dados.leituras[1] = 2.0f;
	envelope.conteudo.bloco_dados.leituras[2] = 3.0f;
	envelope.conteudo.bloco_dados.leituras[3] = 4.0f;
	envelope.conteudo.bloco_dados.leituras[4] = 5.0f;
	envelope.conteudo.bloco_dados.leituras[5] = 6.0f;
	envelope.conteudo.bloco_dados.leituras[6] = 7.0f;
	envelope.conteudo.bloco_dados.leituras[7] = 8.0f;
	envelope.conteudo.bloco_dados.leituras[8] = 9.0f;
	envelope.conteudo.bloco_dados.leituras[9] = 10.0f;
	envelope.conteudo.bloco_dados.leituras_count = 10;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer_protobuf, sizeof(buffer_protobuf));
    if (pb_encode(&stream, MensagemSerial_fields, &envelope)) {
        size_t tamanho_protobuf = stream.bytes_written;

        // Calcula o CRC32 sobre os bytes que o Protobuf acabou de gerar
        uint32_t crc = crc32_calculate(buffer_protobuf, tamanho_protobuf);

        // Injeta o CRC32 (4 bytes) como LITTLE ENDIAN (Byte menos significativo primeiro)
        buffer_protobuf[tamanho_protobuf + 0] = (uint8_t)(crc & 0xFF);         // LSB
		buffer_protobuf[tamanho_protobuf + 1] = (uint8_t)((crc >> 8) & 0xFF);
		buffer_protobuf[tamanho_protobuf + 2] = (uint8_t)((crc >> 16) & 0xFF);
		buffer_protobuf[tamanho_protobuf + 3] = (uint8_t)((crc >> 24) & 0xFF); // MSB

		tamanho_protobuf += 4;

        // Codifica com COBS (garante que não haverá 0x00 nos dados)
        size_t tamanho_cobs = cobs_encode(buffer_protobuf, tamanho_protobuf, buffer_cobs);

        // Adiciona o marcador de fim de pacote
        buffer_cobs[tamanho_cobs] = 0x00;
        tamanho_cobs++;

        // Envia pela Serial/UART (Ex: HAL_UART_Transmit, Serial.write...)
        tud_cdc_send(buffer_cobs, tamanho_cobs, portMAX_DELAY);
    }
}

void enviar_climate_data(void) {
	MensagemSerial envelope = MensagemSerial_init_zero;

	envelope.which_conteudo = MensagemSerial_status_sistema_tag;

	envelope.conteudo.status_sistema.timestamp = xTaskGetTickCount();
	envelope.conteudo.status_sistema.temperatura = 230;
	envelope.conteudo.status_sistema.umidade = 34;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer_protobuf, sizeof(buffer_protobuf));
    if (pb_encode(&stream, MensagemSerial_fields, &envelope)) {
        size_t tamanho_protobuf = stream.bytes_written;

        // Calcula o CRC32 sobre os bytes que o Protobuf acabou de gerar
        uint32_t crc = crc32_calculate(buffer_protobuf, tamanho_protobuf);

        // Injeta o CRC32 (4 bytes) como LITTLE ENDIAN (Byte menos significativo primeiro)
        buffer_protobuf[tamanho_protobuf + 0] = (uint8_t)(crc & 0xFF);         // LSB
		buffer_protobuf[tamanho_protobuf + 1] = (uint8_t)((crc >> 8) & 0xFF);
		buffer_protobuf[tamanho_protobuf + 2] = (uint8_t)((crc >> 16) & 0xFF);
		buffer_protobuf[tamanho_protobuf + 3] = (uint8_t)((crc >> 24) & 0xFF); // MSB

		tamanho_protobuf += 4;

        // Codifica com COBS (garante que não haverá 0x00 nos dados)
        size_t tamanho_cobs = cobs_encode(buffer_protobuf, tamanho_protobuf, buffer_cobs);

        // Adiciona o marcador de fim de pacote
        buffer_cobs[tamanho_cobs] = 0x00;
        tamanho_cobs++;

        // Envia pela Serial/UART (Ex: HAL_UART_Transmit, Serial.write...)
        tud_cdc_send(buffer_cobs, tamanho_cobs, portMAX_DELAY);
    }
}
//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void* params)
{
  (void) params;
  //uint8_t buffer[16];
  uint32_t idx = 0;
  cdc_rx_queue = xQueueCreate(8, sizeof(uint32_t));
  cdc_tx_sem = xSemaphoreCreateBinary();

	do {
		vTaskDelay(10);
	}while (!tud_cdc_connected());

  // RTOS forever loop
  while ( 1 )
  {
#if 0
		/* This implementation reads a single character at a time.  Wait in the
		Blocked state until a character is received. */
		// read
		uint32_t count = tud_cdc_receive(buffer, 1, portMAX_DELAY);

        // read and echo back
		if (count){
			tud_cdc_send(buffer, count, portMAX_DELAY);
		}
#endif
		enviar_dados_massivos(idx++);
		vTaskDelay(1000);
		enviar_climate_data();
		vTaskDelay(1000);
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
