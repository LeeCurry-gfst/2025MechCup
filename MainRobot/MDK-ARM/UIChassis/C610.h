#ifndef __C610_h
#define __C610_h
/*--------------------------- can_driver.h -----------------------------*/


/* ���IDӳ�䣨����ʵ�ʽ��ߵ�����*/
typedef enum {
    C610_1_ID = 0x201,  // ���1��׼ID
    C610_2_ID = 0x202,  // ���2��׼ID
    C610_3_ID = 0x203,  // ���3��׼ID
    C610_4_ID = 0x204   // ���4��׼ID
} C610_ID;

struct M2006_Para{
	float Speed_rpm;
	float Angle_Now;
	float Angle_Last;
	int8_t Round_Count;
	uint8_t Rx_Buffer[8];
	int Target;//���ڵ���
};

extern struct M2006_Para C610[];
/* �������� */
void C610_Init(void);

uint8_t  CAN_Send_Message(uint32_t id,uint8_t *Data,uint8_t Len);
uint8_t CAN_Receive_Message(CAN_RxHeaderTypeDef *can1rxh,uint8_t *data);
void Chassis_analysis(uint8_t i);

#endif
