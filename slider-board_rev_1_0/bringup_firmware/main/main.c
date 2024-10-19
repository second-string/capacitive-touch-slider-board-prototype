#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_app_trace.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define LED_SR_DATA (GPIO_NUM_25)
#define LED_SR_CLOCK (GPIO_NUM_26)
#define LED_SR_LATCH (GPIO_NUM_27)
#define LED_SR_ALL_ON (0x0001FFFF)
#define LED_SR_OFFSETS (0x0000AAAA)
#define LED_SR_ALL_OFF (0x00000000)

#define I2C_SCL_1 (GPIO_NUM_21)
#define I2C_SDA_1 (GPIO_NUM_2)
#define I2C_EXPECT_ACK (true)
#define AT42QT2120_I2C_ADDR (0x1C)
#define AT42QT2160_THICK_I2C_ADDR (0x0D)
#define AT42QT2160_THIN_I2C_ADDR (0x17)

#define CAP_TOUCH_CHANGE_0 (GPIO_NUM_17)  // 2120
#define CAP_TOUCH_CHANGE_1 (GPIO_NUM_18)  // 2160 thin
#define CAP_TOUCH_CHANGE_2 (GPIO_NUM_19)  // 2160 thick

/*
#define DMX_TX_PIN (GPIO_NUM_21)
#define DMX_RX_PIN (GPIO_NUM_17)
#define DMX_DE_nRE_PIN (GPIO_NUM_18)
*/

#define TAG "main"

typedef struct {
    uint8_t slider_pos;
    bool    slider_detected;
    bool    key0_detected;
    bool    key1_detected;
    bool    key2_detected;
} at42qt2120_key_status_t;

static volatile bool    cap_touch_change_flag;
static volatile uint8_t cap_touch_change_mask;

static void IRAM_ATTR cap_touch_change_isr(void *arg) {
    uint32_t pin = (uint32_t)arg;
    if (pin == CAP_TOUCH_CHANGE_0) {
        cap_touch_change_mask |= 1 << 0;
    } else if (pin == CAP_TOUCH_CHANGE_1) {
        cap_touch_change_mask |= 1 << 1;
    } else if (pin == CAP_TOUCH_CHANGE_2) {
        cap_touch_change_mask |= 1 << 2;
    } else {
        configASSERT(0);
    }

    cap_touch_change_flag = true;
}

static uint16_t slider_pos_to_led_mask(uint8_t slider_pos) {
    uint16_t mask = 0;

    // Doesn't care about higher row of LEDs at all
    uint8_t num_leds = (slider_pos / 32) + 1;
    for (uint8_t i = 0; i < num_leds; i++) {
        mask |= (1 << i);
    }

    return mask;
}

static void led_sr_send(uint32_t mask) {
    gpio_set_level(LED_SR_CLOCK, 0);
    for (uint8_t i = 0; i < 16; i++) {
        gpio_set_level(LED_SR_DATA, (1 << (16 - i - 1) & mask));
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(LED_SR_CLOCK, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(LED_SR_CLOCK, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(LED_SR_LATCH, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(LED_SR_LATCH, 0);
}

static uint8_t i2c_read_byte(uint8_t device_addr, uint8_t mem_addr) {
    i2c_cmd_handle_t i2c_cmd_handle = {0};
    uint8_t          byte           = 0x00;

    // Write internal register addr of key status byte 2 with write bit
    // Build command within cmd handle - nothing is transmitted at this point
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (device_addr << 1) | I2C_MASTER_WRITE,
                          I2C_EXPECT_ACK);                            // addr+wr
    i2c_master_write_byte(i2c_cmd_handle, mem_addr, I2C_EXPECT_ACK);  // mem_addr in slave
    i2c_master_stop(i2c_cmd_handle);

    // Transmit constructed cmd sequence
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));

    // Tear down constructed cmd handle for that specific cmd
    i2c_cmd_link_delete(i2c_cmd_handle);

    // Read internal register of key status 0 with read bit
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (device_addr << 1) | I2C_MASTER_READ, I2C_EXPECT_ACK);  // addr+re
    i2c_master_read_byte(i2c_cmd_handle, &byte, I2C_MASTER_LAST_NACK);
    i2c_master_stop(i2c_cmd_handle);

    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    // ESP_LOGI(TAG, "addr: 0x%02X = 0x%02X", mem_addr, byte);

    return byte;
}

static void i2c_write_byte(uint8_t device_addr, uint8_t mem_addr, uint8_t value) {
    i2c_cmd_handle_t i2c_cmd_handle = {0};

    // Build command within cmd handle - nothing is transmitted at this point
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (device_addr << 1) | I2C_MASTER_WRITE,
                          I2C_EXPECT_ACK);                            // addr+wr
    i2c_master_write_byte(i2c_cmd_handle, mem_addr, I2C_EXPECT_ACK);  // mem_addr in slave
    i2c_master_write_byte(i2c_cmd_handle, value, I2C_EXPECT_ACK);
    i2c_master_stop(i2c_cmd_handle);

    // Transmit constructed cmd sequence
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));

    // Tear down constructed cmd handle for that specific cmd
    i2c_cmd_link_delete(i2c_cmd_handle);
}

static void cap_touch_2120_read_status_bytes(at42qt2120_key_status_t *key_status) {
    uint8_t key_status_0  = i2c_read_byte(AT42QT2120_I2C_ADDR, 0x03);
    uint8_t key_status_1  = i2c_read_byte(AT42QT2120_I2C_ADDR, 0x04);
    uint8_t detect_status = i2c_read_byte(AT42QT2120_I2C_ADDR, 0x02);
    uint8_t slider_pos    = i2c_read_byte(AT42QT2120_I2C_ADDR, 0x05);

    (void)key_status_1;

    if (detect_status & (1 << 1)) {
        // Bit 1 SDET set if slider touch detected (also always sets bit 0 TDET, ignore to prioritize slider)
        key_status->slider_detected = true;
        key_status->slider_pos      = slider_pos;
    } else if (detect_status & (1 << 0)) {
        // Bit 0 TDET set if touch on any key detected
        if (key_status_0 & (1 << 3)) {
            key_status->key0_detected = true;
        } else if (key_status_0 & (1 << 4)) {
            key_status->key1_detected = true;
        } else if (key_status_0 & (1 << 5)) {
            key_status->key2_detected = true;
        }
    }

    ESP_LOGI(TAG, "2120      : Detect status: 0x%02X, Slider pos: 0x%02X", detect_status, slider_pos);
}

static uint8_t cap_touch_2160_thin_read_status_bytes() {
    uint8_t key_status_0  = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x03);
    uint8_t key_status_1  = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x04);
    uint8_t detect_status = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x02);
    uint8_t slider_pos    = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x05);
    uint8_t gpio_input    = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x06);

    (void)key_status_0;
    (void)key_status_1;
    (void)detect_status;
    (void)gpio_input;
    // ESP_LOGI(TAG,
    //          "2160 thin : Detect status: 0x%02X, Slider pos: 0x%02X, GPIO: 0x%02X",
    //          detect_status,
    //          slider_pos,
    //          gpio_input);
    return slider_pos;
}

static uint8_t cap_touch_2160_thick_read_status_bytes() {
    uint8_t key_status_0  = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x03);
    uint8_t key_status_1  = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x04);
    uint8_t detect_status = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x02);
    uint8_t slider_pos    = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x05);
    uint8_t gpio_input    = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x06);

    (void)key_status_0;
    (void)key_status_1;
    (void)detect_status;
    (void)gpio_input;
    // ESP_LOGI(TAG,
    //          "2160 thick: Detect status: 0x%02X, Slider pos: 0x%02X, GPIO: 0x%02X",
    //          detect_status,
    //          slider_pos,
    //          gpio_input);
    return slider_pos;
}

static void system_init(void) {
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
}

static void system_start(void) {
}

static void app_init() {
    cap_touch_change_flag = false;
    cap_touch_change_mask = 0x00;

    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << LED_SR_DATA) | (1ULL << LED_SR_CLOCK) | (1ULL << LED_SR_LATCH),
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = false,
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));

    gpio_config_t input_interrupt_config = {
        .pin_bit_mask = (1ULL << CAP_TOUCH_CHANGE_0) | (1ULL << CAP_TOUCH_CHANGE_1) | (1ULL << CAP_TOUCH_CHANGE_2),
        .intr_type    = GPIO_INTR_NEGEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = false,  // pulled up externally
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&input_interrupt_config));
    gpio_install_isr_service(0x0);
    gpio_isr_handler_add(CAP_TOUCH_CHANGE_0, cap_touch_change_isr, (void *)CAP_TOUCH_CHANGE_0);
    gpio_isr_handler_add(CAP_TOUCH_CHANGE_1, cap_touch_change_isr, (void *)CAP_TOUCH_CHANGE_1);
    gpio_isr_handler_add(CAP_TOUCH_CHANGE_2, cap_touch_change_isr, (void *)CAP_TOUCH_CHANGE_2);

    i2c_config_t i2c_config = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_1,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_io_num       = I2C_SCL_1,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000,
        .clk_flags        = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

static void app_start(void) {
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting cap-touch-slider-board firmware");

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    system_init();
    system_start();

    app_init();
    app_start();

    // Enable slider for 2120
    ESP_LOGI(TAG, "********* SETTING UP 2120 *********");
    uint8_t slider_options_val = i2c_read_byte(AT42QT2120_I2C_ADDR, 0x0E);
    ESP_LOGI(TAG, "2120 slider options reg PRE: 0x%02X", slider_options_val);
    i2c_write_byte(AT42QT2120_I2C_ADDR, 0x0E, (1 << 7));  // enable slider
    slider_options_val = i2c_read_byte(AT42QT2120_I2C_ADDR, 0x0E);
    ESP_LOGI(TAG, "2120 slider options reg POST: 0x%02X", slider_options_val);

    ESP_LOGI(TAG, "********* SETTING UP 2160 THICK *********");
    // Set resolution for 2160 thick
    slider_options_val = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x15);
    ESP_LOGI(TAG, "2160 thick slider options reg PRE: 0x%02X", slider_options_val);
    i2c_write_byte(AT42QT2160_THICK_I2C_ADDR, 0x15, 0x00);  // 8 bits resolution
    slider_options_val = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x15);
    ESP_LOGI(TAG, "2160 thick slider options reg POST: 0x%02X", slider_options_val);

    // Set num keys for 2160 thick (leave hysteresis top nibble alone)
    uint8_t slider_control_val = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x14);
    ESP_LOGI(TAG, "2160 thick Slider control reg PRE: 0x%02X", slider_control_val);
    i2c_write_byte(AT42QT2160_THICK_I2C_ADDR, 0x14, (slider_control_val | 0x07));
    slider_options_val = i2c_read_byte(AT42QT2160_THICK_I2C_ADDR, 0x14);
    ESP_LOGI(TAG, "2160 thick slider control reg POST: 0x%02X", slider_control_val);

    // Calibrate 2160 thick after setup
    i2c_write_byte(AT42QT2160_THICK_I2C_ADDR, 0x0A, 0xFF);
    ESP_LOGI(TAG, "Kicked calibration for 2160 thick");

    ESP_LOGI(TAG, "********* SETTING UP 2160 THIN *********");
    slider_options_val = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x15);
    ESP_LOGI(TAG, "2160 thin slider options reg PRE: 0x%02X", slider_options_val);
    i2c_write_byte(AT42QT2160_THIN_I2C_ADDR, 0x15, 0x00);  // 8 bits resolution
    slider_options_val = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x15);
    ESP_LOGI(TAG, "2160 thin slider options reg POST: 0x%02X", slider_options_val);

    // Set num keys for 2160 thin (leave hysteresis top nibble alone)
    slider_control_val = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x14);
    ESP_LOGI(TAG, "2160 thin Slider control reg PRE: 0x%02X", slider_control_val);
    i2c_write_byte(AT42QT2160_THIN_I2C_ADDR, 0x14, (slider_control_val | 0x07));
    slider_options_val = i2c_read_byte(AT42QT2160_THIN_I2C_ADDR, 0x14);
    ESP_LOGI(TAG, "2160 thin slider control reg POST: 0x%02X", slider_control_val);

    // Calibrate 2160 thin after setup
    i2c_write_byte(AT42QT2160_THIN_I2C_ADDR, 0x0A, 0xFF);
    ESP_LOGI(TAG, "Kicked calibration for 2160 thin");

    vTaskDelay(pdMS_TO_TICKS(10));

    at42qt2120_key_status_t key_status = {0};
    (void)cap_touch_2120_read_status_bytes(&key_status);
    (void)cap_touch_2160_thick_read_status_bytes();
    (void)cap_touch_2160_thin_read_status_bytes();
    memset(&key_status, 0x00, sizeof(at42qt2120_key_status_t));

    // TODO :: set gpios all outputs for 2160, otherwise the gpio1 TP triggers capacitively

    uint16_t led_mask = LED_SR_OFFSETS;
    led_sr_send(led_mask);
    ESP_LOGI(TAG, "Sent SR bytes");

    uint8_t slider_pos = 0;
    while (1) {
        if (cap_touch_change_flag) {
            cap_touch_change_flag = false;
            // ESP_LOGW(TAG, "cap touch change flag! mask: %u", cap_touch_change_mask);

            if (cap_touch_change_mask & (1 << 0)) {
                cap_touch_2120_read_status_bytes(&key_status);
                if (key_status.slider_detected) {
                    slider_pos = key_status.slider_pos;
                } else if (key_status.key0_detected || key_status.key1_detected || key_status.key2_detected) {
                    led_sr_send(LED_SR_ALL_OFF);
                }

                memset(&key_status, 0x00, sizeof(at42qt2120_key_status_t));
            }

            if (cap_touch_change_mask & (1 << 1)) {
                slider_pos = cap_touch_2160_thin_read_status_bytes();
            }

            if (cap_touch_change_mask & (1 << 2)) {
                slider_pos = cap_touch_2160_thick_read_status_bytes();
            }

            if (slider_pos != 0) {
                led_mask = slider_pos_to_led_mask(slider_pos);

                // ESP_LOGI(TAG, "Sending slider pos: %u, mask: 0x%02X", slider_pos, led_mask);
                led_sr_send(led_mask);
                slider_pos = 0;
            }

            cap_touch_change_mask = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
