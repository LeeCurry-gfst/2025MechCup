#ifndef __Chassis_h
#define __Chassis_h

/*底盘电机控制位宏定义*/
#define GPIO_PIDdebugFlag GPIO_KEY1
#define GPIO_PIDdebugFlag_PIN GPIO_KEY1_PIN

/*运动学解算参数宏定义*/
/*运动学解算统一单位cm,s,°*/
#define Chassis_Width 26.0f//本车宽度，按车轮间距算，单位:cm
#define Chassis_Length 21.3f//本车长度，按车轮间距算，单位:cm
#define Chassis_WheelR 6.35f //车轮半径

/*最大速度调控参数宏定义*/
/*运动学解算统一单位cm,s,°*/
#define Chassis_ForwardMax 10.0f //100cm/s
#define Chassis_rotateMax 20.f //60°/s

void Chassis_anglePID_Calculation(void);
void Chassis_TeleCommandExecute(void);
void Chassis_PID_Init(void);

enum Chassis_KpidConditionFlag
{
	Normal=0,
	Upstair,
	Uphill,
	Debug,
	OFF
};

struct Chassis_PID{
	float Actual;
	float Target;
	float Torque;
	float Kp,Ki,Kd;
	float Kpus,Kius,Kdus;//上台阶PID参数
	float Kpuh,Kiuh,Kduh;//上坡PID参数
	float ErrorNow,ErrorLast,ErrorSum;
	float ErrorLim,ErrorSumLim;
	float Out;
	float dt;//控制单位时间，单位s
	enum Chassis_KpidConditionFlag KpidFlag;//目前所处的路况，分为正常上坡上台阶调试关闭
};


extern uint32_t Kpid[];
extern struct Chassis_PID wheel[];
extern struct Chassis_PID angle;


#endif
