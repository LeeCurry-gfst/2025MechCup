#ifndef __Chassis_h
#define __Chassis_h

/*���̵������λ�궨��*/
#define GPIO_PIDdebugFlag GPIO_KEY1
#define GPIO_PIDdebugFlag_PIN GPIO_KEY1_PIN

/*�˶�ѧ��������궨��*/
/*�˶�ѧ����ͳһ��λcm,s,��*/
#define Chassis_Width 26.0f//������ȣ������ּ���㣬��λ:cm
#define Chassis_Length 21.3f//�������ȣ������ּ���㣬��λ:cm
#define Chassis_WheelR 6.35f //���ְ뾶

/*����ٶȵ��ز����궨��*/
/*�˶�ѧ����ͳһ��λcm,s,��*/
#define Chassis_ForwardMax 10.0f //100cm/s
#define Chassis_rotateMax 20.f //60��/s

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
	float Kpus,Kius,Kdus;//��̨��PID����
	float Kpuh,Kiuh,Kduh;//����PID����
	float ErrorNow,ErrorLast,ErrorSum;
	float ErrorLim,ErrorSumLim;
	float Out;
	float dt;//���Ƶ�λʱ�䣬��λs
	enum Chassis_KpidConditionFlag KpidFlag;//Ŀǰ������·������Ϊ����������̨�׵��Թر�
};


extern uint32_t Kpid[];
extern struct Chassis_PID wheel[];
extern struct Chassis_PID angle;


#endif
