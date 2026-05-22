#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"

#define LASER_PIN 4
#define LASER_BTN 15
#define DATA_SEND_BTN 19
#define LASER_RECEPTOR_PIN 5
#define MS100 pdMS_TO_TICKS(100)
#define LASER_SIGNAL_MARGIN pdMS_TO_TICKS(30)
#define HIGH_TIME pdMS_TO_TICKS(10)
#define LOW_TIME pdMS_TO_TICKS(20)
#define HIGH_TIME_MS 10
#define LOW_TIME_MS 20
#define NOISE_THRESHOLD_MS 1
#define REGISTER_BTN_0 26
#define REGISTER_BTN_4 27

static const uint8_t OUTPUT_REGISTER_PINS[8] = {22, 23, 21, 18, 12, 13, 14, 25};
static QueueHandle_t event_queue = NULL;
static QueueHandle_t rmt_receive_event_queue = NULL;
volatile bool is_data_send_task_active = false;
volatile uint8_t laser_level = 0;
static volatile uint8_t data8 = 0;

static void display_data_on_led(uint32_t data_to_display) {
    for (int16_t i = 7; i > -1; i--) {
        uint8_t bit = (data_to_display >> (7-i)) & 1UL;
        gpio_set_level(OUTPUT_REGISTER_PINS[i], bit);
    }
}

static bool IRAM_ATTR rmt_receive_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;

    xQueueSendFromISR(queue, edata, &high_task_wakeup);

    return high_task_wakeup == pdTRUE;
}

static void task_laser_receptor_listen(void* pvParamaters) {
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LASER_RECEPTOR_PIN,
        .resolution_hz = 500000,
        .mem_block_symbols = 128,
    };

    rmt_channel_handle_t rx_chan = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_receive_callback
    };

    rmt_receive_event_queue = xQueueCreate(10, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_register_event_callbacks(rx_chan, &cbs, rmt_receive_event_queue);

    ESP_ERROR_CHECK(rmt_enable(rx_chan));
    rmt_symbol_word_t raw_symbols[64];
    rmt_receive_config_t receive_config = { 
        .signal_range_min_ns = 3000,
        .signal_range_max_ns = 60000000
    }; 

    rmt_rx_done_event_data_t rx_data;
    while (1) {
        ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));
        
        if (xQueueReceive(rmt_receive_event_queue, &rx_data, portMAX_DELAY)) {
            uint32_t data_received_buffer = 0;
            uint8_t bit_count = 0;
            uint16_t symbols_amount = rx_data.num_symbols;

            ESP_LOGI("SYMBOLS_RCEIVED", "%u", symbols_amount);
            
            for (int i = 0; i < symbols_amount; i++) {                
                rmt_symbol_word_t symbol = rx_data.received_symbols[i];
                
                // RMT symbols have two halves (duration0/level0 and duration1/level1)
                uint32_t duration = symbol.duration0;
                uint32_t level = symbol.level0;

                uint32_t duration_ms = duration * 2/1000;
                uint8_t data_lvl = 0;
                if (level == 0) { 
                    if (duration_ms >= (HIGH_TIME_MS - NOISE_THRESHOLD_MS) && duration_ms <= (HIGH_TIME_MS + NOISE_THRESHOLD_MS)) {
                        data_received_buffer |= (1UL << bit_count);
                        data_lvl = 1;
                        bit_count++;
                    } 
                    else if (duration_ms >= (LOW_TIME_MS - NOISE_THRESHOLD_MS) && duration_ms <= (LOW_TIME_MS + NOISE_THRESHOLD_MS)) {
                        data_received_buffer &= ~(1UL << bit_count);
                        data_lvl = 0;
                        bit_count++;
                    }
                    
                    if (bit_count >= 32) break;
                }
                printf("Symbol[%d]: Level=%u, Duration=%lu ticks (%lu ms)\n", 
                        i, 
                        data_lvl, 
                        duration, 
                        duration_ms);
            }

            if (bit_count > 0) {
                ESP_LOGI("RECEIVED DATA", "Value: %lu | Bits: %u", data_received_buffer, bit_count);
                display_data_on_led(data_received_buffer);
            }
            else ESP_LOGI("RECEIVED DATA", "No data was received");
        }
    }
}

static void task_send_data(void* pvParameters) {
    uint32_t data = *(uint32_t*) pvParameters; 
    free(pvParameters);

    uint8_t was_on = laser_level;

    gpio_set_level(LASER_PIN, 0);
    vTaskDelay(LASER_SIGNAL_MARGIN);
   
    uint8_t horl = -1;
    for (uint8_t i = 0; i < 32; i++) {
        horl = (data >> i) & 0x00000001;
        
        gpio_set_level(LASER_PIN, 1);
        if (horl) vTaskDelay(HIGH_TIME);
        else vTaskDelay(LOW_TIME);

        // bit_gap
        gpio_set_level(LASER_PIN, 0);
        vTaskDelay(LASER_SIGNAL_MARGIN);
    }

    gpio_set_level(LASER_PIN, was_on);
    is_data_send_task_active = false;
    vTaskDelete(NULL);
}

static volatile uint32_t last_intr_tick = 0;
static void IRAM_ATTR intr_handler(void* arg) {
    uint32_t latest_intr_tick = xTaskGetTickCountFromISR();
    if (latest_intr_tick - last_intr_tick < 20) {
        return;
    }
    last_intr_tick = latest_intr_tick;
    uint32_t pin_no = (uint32_t)arg;
    xQueueSendFromISR(event_queue, &pin_no, NULL);
}

static void event_handler(void *pvParameters) {
    uint32_t pin_no = 0;
    uint8_t x = 0;
    while(1){
        if (xQueueReceive(event_queue, &pin_no, portMAX_DELAY)) {
            switch (pin_no) {
                case LASER_RECEPTOR_PIN:
                    x++;
                    //ESP_LOGI("LASER_RECEPTOR", "Triggered %u", (unsigned int) x);
                    break;
                    
                case LASER_BTN:
                    ESP_LOGI("register size", "%u", (unsigned int)sizeof(gpio_get_level(LASER_PIN)));
                    laser_level ^= 0x01;
                    gpio_set_level(LASER_PIN, laser_level);
                    break;
                
                case DATA_SEND_BTN:
                    if (!is_data_send_task_active) {
                        uint32_t* pxData = (uint32_t*) malloc(sizeof(uint32_t));
                        *pxData = (uint32_t) data8;

                        esp_err_t ret = xTaskCreate(task_send_data, "task_send_data", 2048, (void*) pxData, 10, NULL);
                        if (ret != pdPASS) {
                            ESP_LOGI("TASK_CREATION_RETURN", "Failed.");
                            free(pxData);
                            is_data_send_task_active = false;
                        } else is_data_send_task_active = true;
                    }
                    break;
                
                case REGISTER_BTN_0:
                    data8 += 0x10;
                    display_data_on_led(data8);
                    break;
                    
                    case REGISTER_BTN_4:
                    data8 += 0x01;
                    display_data_on_led(data8);
                    break;
            }
        }
    }
}

void app_main(void)
{
    event_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t o_config = {
        .pin_bit_mask = (1ULL << LASER_PIN) | (1ULL << OUTPUT_REGISTER_PINS[0]) | (1ULL << OUTPUT_REGISTER_PINS[1]) | (1ULL << OUTPUT_REGISTER_PINS[2]) | (1ULL << OUTPUT_REGISTER_PINS[3]) | (1ULL << OUTPUT_REGISTER_PINS[4]) | (1ULL << OUTPUT_REGISTER_PINS[5]) | (1ULL << OUTPUT_REGISTER_PINS[6]) | (1ULL << OUTPUT_REGISTER_PINS[7]),
        // .pin_bitmask = 0b0000000000000000000000000000000000000010111001000111000000010000ULL, // Uncomment this line and comment out the above for faster execution.
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE, 
        .pull_down_en = 0,
        .pull_up_en = 0
    };
     
    gpio_config_t i_config = {
        .pin_bit_mask = (1ULL << LASER_BTN) | (1ULL << DATA_SEND_BTN) | (1ULL << REGISTER_BTN_0) | (1ULL << REGISTER_BTN_4),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_down_en = 1,
        .pull_up_en = 0
    };
    
    gpio_config(&o_config);
    gpio_config(&i_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LASER_BTN, intr_handler, (void*)LASER_BTN);
    gpio_isr_handler_add(DATA_SEND_BTN, intr_handler, (void*)DATA_SEND_BTN);
    gpio_isr_handler_add(REGISTER_BTN_0, intr_handler, (void*)REGISTER_BTN_0);
    gpio_isr_handler_add(REGISTER_BTN_4, intr_handler, (void*)REGISTER_BTN_4);

    xTaskCreate(event_handler, "event_handler_task", 4096, NULL, 10, NULL);
    xTaskCreate(task_laser_receptor_listen, "task_laser_receptor_listen", 4096, NULL, 10, NULL);
    while(1) {
        vTaskDelay(100);
    }
}