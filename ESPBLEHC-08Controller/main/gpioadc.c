#include "driver/gpio.h"  // 包含GPIO驱动库
#include "esp_log.h"  // 用于日志打印
#include "freertos/FreeRTOS.h"  // 用于FreeRTOS
#include "freertos/task.h"  // 用于任务创建和延时
#include "driver/adc.h"   // 包含ADC驱动库
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
// ADC通道宏定义
#define ADC_Chassis_X1        ADC1_CHANNEL_0  
#define ADC_Chassis_Y1        ADC1_CHANNEL_1  
#define ADC_Chassis_X2        ADC1_CHANNEL_3
#define ADC_Chassis_Y2        ADC1_CHANNEL_4

// 全局变量：存储每个ADC通道的读取值
int adc_chassis_x1_value = 0;
int adc_chassis_y1_value = 0;
int adc_chassis_x2_value = 0;
int adc_chassis_y2_value = 0;


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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_Chassis_X1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_Chassis_Y1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_Chassis_X2, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_Chassis_Y2, &config));

}

// 读取ADC值的任务
void adc_task(void *arg) {
    while (1) {

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_Chassis_X1, &adc_chassis_x1_value));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_Chassis_Y1, &adc_chassis_y1_value));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_Chassis_X2, &adc_chassis_x2_value));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_Chassis_Y2, &adc_chassis_y2_value));

        // 打印读取的值（可以通过串口监视）
        ESP_LOGI("ADC_READ", "Chassis X1: %d", adc_chassis_x1_value);
        ESP_LOGI("ADC_READ", "Chassis Y1: %d", adc_chassis_y1_value);
        ESP_LOGI("ADC_READ", "Chassis X2: %d", adc_chassis_x2_value);
        ESP_LOGI("ADC_READ", "Chassis Y2: %d", adc_chassis_y2_value);

        vTaskDelay(100 / portTICK_PERIOD_MS);  // 每隔30ms读取一次
    }
}

// GPIO宏定义
#define GPIO_Mode                GPIO_NUM_6  
#define GPIO_Container_Pourout   GPIO_NUM_11  
#define GPIO_Container_Upcup     GPIO_NUM_12  
#define GPIO_Container_Updown    GPIO_NUM_13  
#define GPIO_Cleaner             GPIO_NUM_14  

// 全局变量：保存每个GPIO口的状态
int gpio_mode = 0;//默认上拉高电平为主车
int gpio_container_pourout_state = 0;
int gpio_container_upcup_state = 0;
int gpio_container_updown_state = 0;
int gpio_cleaner = 0;


// 消抖延迟时间（单位：毫秒）
#define DEBOUNCE_DELAY 50

// 初始化GPIO口的配置函数
//机械按键开关全部默认高电平
void gpio_init(void) {

    gpio_config_t io_conf;

    // 配置各个按键为输入，并启用上拉电阻

    io_conf.intr_type = GPIO_INTR_DISABLE;  // 禁用中断
    io_conf.mode = GPIO_MODE_INPUT;  // 设置为输入模式
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // 禁用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // 启用上拉电阻
    
    io_conf.pin_bit_mask = (1ULL << GPIO_Mode);  // 设置GPIO_Mode的bit掩码
    gpio_config(&io_conf);  // 配置GPIO口

    io_conf.pin_bit_mask = (1ULL << GPIO_Container_Pourout);  // 设置GPIO_Container_Pourout的bit掩码
    gpio_config(&io_conf);  // 配置GPIO口

    io_conf.pin_bit_mask = (1ULL << GPIO_Container_Upcup);  // 设置GPIO_Container_Upcup的bit掩码
    gpio_config(&io_conf);  // 配置GPIO口

    io_conf.pin_bit_mask = (1ULL << GPIO_Container_Updown);  // 设置GPIO_Container_Updown的bit掩码
    gpio_config(&io_conf);  // 配置GPIO口

    io_conf.pin_bit_mask = (1ULL << GPIO_Cleaner);  // 设置GPIO_Cleaner的bit掩码
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
        int mode_state = debounce_read(GPIO_Mode);
        int pourout_state = debounce_read(GPIO_Container_Pourout);
        int upcup_state = debounce_read(GPIO_Container_Upcup);
        int updown_state = debounce_read(GPIO_Container_Updown);
        int cleaner_state = debounce_read(GPIO_Cleaner);

        // 如果按键稳定为低电平（按下），更新状态变量
        if (mode_state != -1) {
            gpio_mode = (mode_state == 0) ? 1 : 0; //默认上拉高电平为主车，状态置0
        }
        if (pourout_state != -1) {
            gpio_container_pourout_state = (pourout_state == 1) ? 1 : 0; //未按下为高电平
        }
        if (upcup_state != -1) {
            gpio_container_upcup_state = (upcup_state == 1) ? 1 : 0; //未按下为高电平
        }
        if (updown_state != -1) {
            gpio_container_updown_state = (updown_state == 1) ? 1 : 0; //未按下为高电平
        }
        if (cleaner_state != -1) {
            gpio_cleaner = (cleaner_state == 1) ? 1 : 0; //未按下为高电平
        }
        
        
        ESP_LOGI("GPIO_VALUE", "Container Pourout: %d", pourout_state);
        ESP_LOGI("GPIO_VALUE", "Container Upcup: %d", upcup_state);
        ESP_LOGI("GPIO_VALUE", "Container Updown: %d", updown_state);
        ESP_LOGI("GPIO_VALUE", "Cleaner: %d", cleaner_state);

        // 打印状态，便于调试
        ESP_LOGI("GPIO_STATE", "Mode: %d", gpio_mode);
        ESP_LOGI("GPIO_STATE", "Container Pourout: %d", gpio_container_pourout_state);
        ESP_LOGI("GPIO_STATE", "Container Upcup: %d", gpio_container_upcup_state);
        ESP_LOGI("GPIO_STATE", "Container Updown: %d", gpio_container_updown_state);
        ESP_LOGI("GPIO_STATE", "Cleaner: %d", gpio_cleaner);

        vTaskDelay(100 / portTICK_PERIOD_MS);  // 定时任务延时
    }
}

void get_btData(uint8_t *btData)
{
    //主车遥控解算
    btData[0]=0xAA;

    btData[1]=adc_chassis_x2_value>>4;
    btData[2]=adc_chassis_y2_value>>4;
    btData[3]=gpio_container_pourout_state;
    btData[4]=gpio_container_upcup_state;
    btData[5]=gpio_container_updown_state;
    btData[6]=gpio_cleaner;
    btData[7]=adc_chassis_x1_value>>4;
    
    btData[8]=0xEE;
    ESP_LOGI("GPIO_STATE", "SET BT DATA");
}
