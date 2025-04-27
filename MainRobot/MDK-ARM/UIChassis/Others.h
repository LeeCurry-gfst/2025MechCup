#ifndef __Others_h
#define __Others_h

#define TIM_ARM TIM1//机械臂控制
#define htim_arm htim1
#define TIM_CHANNEL_ARM1 TIM_CHANNEL_1
#define TIM_CHANNEL_ARM2 TIM_CHANNEL_2
#define TIM_CHANNEL_ARM3 TIM_CHANNEL_3
#define TIM_CHANNEL_ARM4 TIM_CHANNEL_4





#define TIM_SUCKER TIM2//吸盘控制
#define htim_sucker htim2
#define TIM_CHANNEL_VALVE TIM_CHANNEL_4//阀门
#define TIM_CHANNEL_PUMP TIM_CHANNEL_3//气泵

#define TIM_LAUNCH TIM3//投放结构
#define htim_launch htim3
#define TIM_CHANNEL_POUROUT TIM_CHANNEL_1//倾斜（倒出）
#define TIM_CHANNEL_DROP TIM_CHANNEL_2//开阀（开盖

#define TIM_STEP TIM5//步进电机控制
#define htim_step htim5
#define TIM_CHANNEL_STEP TIM_CHANNEL_2
#define GPIO_STEP GPIOF
#define GPIO_PIN_DIR GPIO_PIN_7
#define GPIO_PIN_MS1 GPIO_PIN_8
#define GPIO_PIN_MS2 GPIO_PIN_9
#define GPIO_PIN_EN GPIO_PIN_10//（低电平启动）

/*
*MS1| MS2 |细分
0   | 0   |8
1   | 0   |32
0   | 1   |64
1   | 1   |16
*/

#define GPIO_FAN GPIOC//风机继电器控制（高电平启动）
#define GPIO_PIN_FAN GPIO_PIN_5

extern float ARM1_InitAngle;
extern float ARM2_InitAngle;
extern float ARM3_InitAngle;
extern float ARM4_InitAngle;
extern float ARM1_Start;
extern float ARM2_Start;
extern float ARM3_Start;
extern float ARM4_Start;

void Others_Init(void);
#endif
