#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_crt_bundle.h"
#include "esp_partition.h"
#include "protocol_examples_common.h"
#include "esp_crc.h"

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define TAG "SENDER:"
#define START_SIGNAL "START"
#define START_SIGNAL_LEN 5
#define STOP_SIGNAL "STOP"
#define STOP_SIGNAL_LEN 4
#define FIRMWARE_UPGRADE_URL "http://192.168.137.1:8000/Desktop/test/blink/build/blink.bin"

#define TX_PIN 4
#define RX_PIN 5

#define BUFF_SIZE 64
#define SIGNAL_LEN 3
uint8_t *download_data_buffer = NULL;
static int total_packet=0; 


void send_firmware(const uint8_t *data, uint8_t len);

typedef struct{
    uint8_t length;
    uint16_t crc;
    uint16_t packet_number;
    uint16_t total_packets;
    uint8_t data_buffer[BUFF_SIZE];
}data_frame;


QueueHandle_t uart_queue;
uint16_t current_packet =0;

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
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUFF_SIZE * 3, 0, 20, NULL, 0));
}

uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    return esp_crc16_le(0, data, length);
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static int start = 0;
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
                // Check if the header is Content-Length
        if (strcasecmp(evt->header_key, "Content-Length") == 0) {
            total_packet = atoi(evt->header_value);
            ESP_LOGI(TAG, "Total packet length: %d", total_packet);
        }
        break;
    case HTTP_EVENT_ON_DATA:
        char check = '\0';
        current_packet += evt->data_len;
        while(start == 0) {
            ESP_LOGI(TAG, "Sending start message");
            send_firmware((uint8_t*)START_SIGNAL, START_SIGNAL_LEN);
            uint8_t ack_buffer[START_SIGNAL_LEN];
            int ack_len = uart_read_bytes(UART_NUM_1, ack_buffer, START_SIGNAL_LEN, 6000 / portTICK_PERIOD_MS);
            if (ack_len == START_SIGNAL_LEN && strncmp((char *)ack_buffer, START_SIGNAL, START_SIGNAL_LEN) == 0) {
                ESP_LOGI(TAG, "Received start acknowledgment");
                start++;
            } else {
                ESP_LOGE(TAG, "Failed to receive start acknowledgment");
            }
        }
        do{
            data_frame metadata = {0};
            metadata.length = evt->data_len;
            memcpy(metadata.data_buffer,evt->data,evt->data_len);
            uint16_t crc = calculate_crc16(metadata.data_buffer, evt->data_len);
            metadata.crc = crc;
            metadata.packet_number = current_packet;
            metadata.total_packets = total_packet;
            //  for(int x = 0; x < sizeof(data_frame); x++) {
            //             printf("  %02X", *(uint8_t *)((void *)&metadata + x));
            //         }
            //         printf("\n");
            uart_write_bytes(UART_NUM_1,&metadata,sizeof(data_frame));
            uart_wait_tx_done(UART_NUM_1,pdMS_TO_TICKS(100));
            uart_read_bytes(UART_NUM_1,&check, 1, pdMS_TO_TICKS(1000));          

        }while(check == 0);
         
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        static int end=0;
        if (download_data_buffer != NULL) {
            free(download_data_buffer);
            download_data_buffer = NULL;
            while(end==0){
                ESP_LOGI(TAG, "Sending stop signal");
                send_firmware((uint8_t *)STOP_SIGNAL, STOP_SIGNAL_LEN);
                uint8_t ack_buffer[STOP_SIGNAL_LEN];
                int ack_len = uart_read_bytes(UART_NUM_1, ack_buffer, STOP_SIGNAL_LEN, 6000 / portTICK_PERIOD_MS);
                if (ack_len == STOP_SIGNAL_LEN && strncmp((char *)ack_buffer, STOP_SIGNAL, STOP_SIGNAL_LEN) == 0) {
                    ESP_LOGI(TAG, "Received stop acknowledgment");
                    end++;
                } else {
                    ESP_LOGE(TAG, "Failed to receive stop acknowledgment");
                }
            }
        }
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        if (download_data_buffer) {
            free(download_data_buffer);
            download_data_buffer = NULL;
        }
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

void send_firmware(const uint8_t *data, uint8_t len){
    data_frame metadata = {0};
    metadata.length = len;
    memcpy(metadata.data_buffer,data,len);
    uint16_t crc = calculate_crc16(metadata.data_buffer, len);
    metadata.crc = crc;
    metadata.packet_number = current_packet;
    metadata.total_packets = total_packet;
    uart_write_bytes(UART_NUM_1,&metadata,sizeof(data_frame));
    uart_wait_tx_done(UART_NUM_1,pdMS_TO_TICKS(100));
}

void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

void download_firmware(void *pvParameter)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    esp_http_client_config_t config = {
        .url = FIRMWARE_UPGRADE_URL,
        .event_handler = _http_event_handler,
        .buffer_size = BUFF_SIZE,
#ifdef CONFIG_EXAMPLE_USE_CERT_BUNDLE
        .crt_bundle_attach = esp_crt_bundle_attach,
#else
        .cert_pem = (char *)server_cert_pem_start,
#endif
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %" PRId64,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    http_cleanup(client);

    ESP_ERROR_CHECK(example_disconnect());
    vTaskDelete(NULL);
}


void app_main(void){
    uart_init();
    xTaskCreate(download_firmware, "download_firmware_task", 4096, NULL, 7, NULL);
}
