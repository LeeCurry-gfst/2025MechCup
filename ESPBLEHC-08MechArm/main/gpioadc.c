#include "driver/gpio.h"  // 包含GPIO驱动库
#include "esp_log.h"  // 用于日志打印
#include "freertos/FreeRTOS.h"  // 用于FreeRTOS
#include "freertos/task.h"  // 用于任务创建和延时
#include "driver/adc.h"   // 包含ADC驱动库
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"

// ADC通道宏定义
#define ADC_Arm1        ADC1_CHANNEL_0   
#define ADC_Arm2        ADC1_CHANNEL_1   
#define ADC_Arm3        ADC1_CHANNEL_3
#define ADC_Arm4        ADC1_CHANNEL_4   

// 全局变量：存储每个ADC通道的读取值
int adc_arm1_value = 0;
int adc_arm2_value = 0;
int adc_arm3_value = 0;
int adc_arm4_value = 0;


adc_oneshot_unit_handle_t adc1_handle;
void adc_init(void) {
    
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_Arm1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_Arm2, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_Arm3, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_Arm4, &config));
    

}

// 读取ADC值的任务
void adc_task(void *arg) {
    while (1) {

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_Arm1, &adc_arm1_value));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_Arm2, &adc_arm2_value));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_Arm3, &adc_arm3_value));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_Arm4, &adc_arm4_value));

        // 打印读取的值（可以通过串口监视）
        ESP_LOGI("ADC_READ", "ARM1: %d", adc_arm1_value);
        ESP_LOGI("ADC_READ", "ARM2: %d", adc_arm2_value);
        ESP_LOGI("ADC_READ", "ARM3: %d", adc_arm3_value);
        ESP_LOGI("ADC_READ", "ARM4: %d", adc_arm4_value);

        vTaskDelay(50 / portTICK_PERIOD_MS);  // 每隔50ms读取一次
    }
}

// GPIO宏定义
#define GPIO_Arm5                GPIO_NUM_6

// 全局变量：保存每个GPIO口的状态
int gpio_arm5 = 0;//按键默认弹起，为高电平

// 消抖延迟时间（单位：毫秒）
#define DEBOUNCE_DELAY 50

// 初始化GPIO口的配置函数
//机械按键开关全部默认高电平
void gpio_init(void) {

    gpio_config_t io_conf;

    // 配置各个按键为输入，并启用下拉电阻

    io_conf.intr_type = GPIO_INTR_DISABLE;  // 禁用中断
    io_conf.mode = GPIO_MODE_INPUT;  // 设置为输入模式
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;  // 禁用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;  // 启用上拉电阻
    
    io_conf.pin_bit_mask = (1ULL << GPIO_Arm5);  // 设置GPIO_Mode的bit掩码
    gpio_config(&io_conf);  // 配置GPIO口

}

// 消抖函数
bool debounce_read(gpio_num_t gpio_num) {
    int last_state = gpio_get_level(gpio_num);
    vTaskDelay(DEBOUNCE_DELAY / portTICK_PERIOD_MS);  // 延时以进行消抖
    int current_state = gpio_get_level(gpio_num);
    if (last_state == current_state) {
        return current_state;
    } else {
        return -1;  // 如果状态不稳定，返回-1
    }
}

// 任务函数，用于定期检查GPIO口状态
void gpio_task(void *arg) {
    while (1) {
        // 读取每个GPIO的状态，并进行消抖
        int arm5_state = debounce_read(GPIO_Arm5);

        // 如果按键稳定为低电平（按下），更新状态变量
        if (arm5_state != -1) {
            gpio_arm5 = (arm5_state == 0) ? 1 : 0; //默认上拉高电平为主车，状态置0
        }

        // 打印状态，便于调试
        ESP_LOGI("GPIO_STATE", "Arm5: %d", gpio_arm5);

        vTaskDelay(100 / portTICK_PERIOD_MS);  // 定时任务延时
    }
}


void get_btData(uint8_t *btData)
{
    btData[0]=0xAA;

    btData[1]=adc_arm1_value>>8;
    btData[2]=adc_arm1_value&0xff;

    btData[3]=adc_arm2_value>>8;
    btData[4]=adc_arm2_value&0xff;

    btData[5]=adc_arm3_value>>8;
    btData[6]=adc_arm3_value&0xff;

    btData[7]=adc_arm4_value>>8;
    btData[8]=adc_arm4_value&0xff;

    btData[9]=gpio_arm5;
    
    btData[10]=0xEE;
}
