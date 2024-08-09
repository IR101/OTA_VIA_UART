#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "esp_partition.h"
#include "esp_ota_ops.h" 

#define TAG "RECEIVER:"
#define START_SIGNAL "START"
#define START_SIGNAL_LEN 5
#define STOP_SIGNAL "STOP"
#define STOP_SIGNAL_LEN 4


#define TX_PIN 4
#define RX_PIN 5

#define BUFF_SIZE 64
#define SIGNAL_LEN 3

typedef struct{
    uint8_t length;
    uint16_t crc;
    uint16_t packet_number;
    uint16_t total_packets;
    uint8_t data_buffer[BUFF_SIZE];
}data_frame;

QueueHandle_t uart_queue;


void uart_init(){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUFF_SIZE * 3, 0, 20, &uart_queue, 0));
}

uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    return esp_crc16_le(0, data, length);
}

void uart_event_task(void *params){ 
    uart_event_t uart_event;
    esp_err_t err;
    esp_ota_handle_t ota_handle;
    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    static int count=0;
    while (true) {
        if (xQueueReceive(uart_queue, &uart_event, portMAX_DELAY)) {
            data_frame metadata = {0};
            switch (uart_event.type) {
                case UART_DATA: 
                    // Read the metadata and do ota
                int len = uart_read_bytes(UART_NUM_1, &metadata, sizeof(data_frame), 100 / portTICK_PERIOD_MS);
                  if(count!=0){ 
                    if(len == sizeof(data_frame)){
                        err = esp_ota_write(ota_handle,metadata.data_buffer, metadata.length);
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
                            break;
                        }
                        printf("here");
                        char data = 'a';
                        uart_write_bytes(UART_NUM_1, &data, 1);
                        count=10;
                    }
                   }
                    // for(int x = 0; x < sizeof(data_frame); x++) {
                    //     printf("  %02X", *(uint8_t *)((void *)&metadata + x));
                    // }
                    // printf("\n");
                    // // Verify the received CRC
                    uint16_t crc = calculate_crc16(metadata.data_buffer, metadata.length);
                    if (crc != metadata.crc) {
                        ESP_LOGE(TAG, "Calculated crc is: %u and received is: %u",crc,metadata.crc);
                        break; // Exit if CRC does not match
                    }

                    if(memcmp(metadata.data_buffer, START_SIGNAL, START_SIGNAL_LEN) == 0){
                        printf("pass1");
                        ESP_LOGI(TAG, "Received START signal");
                        if (partition == NULL) {
                            ESP_LOGE(TAG, "Failed to find update partition");
                            break;
                        }
                        err = esp_ota_begin(partition, OTA_SIZE_UNKNOWN, &ota_handle);
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
                            break;
                        }
                        uint8_t ack_buffer[START_SIGNAL_LEN];
                        memcpy(ack_buffer,START_SIGNAL,START_SIGNAL_LEN);
                        ESP_LOGI(TAG, "Sending START acknowledgement");
                        uart_write_bytes(UART_NUM_1,ack_buffer,START_SIGNAL_LEN);
                        vTaskDelay(100 /portTICK_PERIOD_MS);
                        count++;
                        printf("pass2");
                        break;
                    }

                    if(memcmp(metadata.data_buffer, STOP_SIGNAL, STOP_SIGNAL_LEN) == 0 || metadata.packet_number==metadata.total_packets){
                        ESP_LOGI(TAG, "Received STOP signal");
                        uint8_t ack_buffer[STOP_SIGNAL_LEN];
                        memcpy(ack_buffer,STOP_SIGNAL,STOP_SIGNAL_LEN);
                        ESP_LOGI(TAG, "Sending STOP acknowledgement");
                        uart_write_bytes(UART_NUM_1, ack_buffer, STOP_SIGNAL_LEN);
                        vTaskDelay(100 /portTICK_PERIOD_MS);
                        err = esp_ota_end(ota_handle);
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
                            break;
                        }

                        err = esp_ota_set_boot_partition(partition);
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
                            break;
                        }
                        ESP_LOGI(TAG, "OTA update successful, restarting...");
                        esp_restart();
                        break;
                    }
                    // Process the received data
                    ESP_LOGI(TAG, "Total packet:  %u Current packet:  %u  Data length: %d   Calculated crc is: %u and received is: %u   Data: %s",metadata.total_packets,metadata.packet_number,metadata.length,crc,metadata.crc,metadata.data_buffer);
                    break;
                case UART_BREAK:
                    ESP_LOGI(TAG, "UART_BREAK");
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "UART_BUFFER_FULL");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "UART_FIFO_OVF");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "UART_FRAME_ERR");
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "UART_PARITY_ERR");
                    break;
                case UART_DATA_BREAK:
                    ESP_LOGI(TAG, "UART_DATA_BREAK");
                    break;
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "UART_PATTERN_DET");
                    break;
                default:
                    ESP_LOGI(TAG, "UART event type: %d", uart_event.type);
                    break;
            }
        }
    }
}



void app_main(void){
    uart_init();
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 7, NULL);
}


