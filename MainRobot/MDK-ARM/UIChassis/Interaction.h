#ifndef __Interaction_h
#define __Interaction_h

void UI_CmpPrintf(void);
void UI_OLEDPrintf(void);
void BLECtrl_RecDataInit(void);



struct BLEctrl{
	float Chassis_S;//ң��Ŀ��X���ٶ�,cm/s
	float Chassis_X;//ң��Ŀ��X���ٶ�,cm/s
	float Chassis_R;//ң��Ŀ����ת�ٶ�,��/s
	
	float Arm1;//ң��Ŀ��ת���Ƕ�,��
	float Arm2;//ң��Ŀ��ת���Ƕ�,��
	float Arm3;//ң��Ŀ��ת���Ƕ�,��
	float Arm4;//ң��Ŀ��ת���Ƕ�,��
	uint8_t Arm5;//ң��Ŀ����ȡ/����
	uint8_t Contianer_updown;//ң�ؿ�������
	uint8_t Pump;//ң�ؿ������ÿ���
	uint8_t Contianer_pourout;//ң�ؿ��Ƶ����������ǣ�
	uint8_t Cleaner;//ң�ؿ��ƿ���������
};
extern struct BLEctrl BLEctrl_Command;



//==============���Դ��ڵ������===============================================

#define huartCmp huart1 //��������ѡ��
#define USARTCmp USART1 //��������ѡ��

extern uint8_t CmpRxDa;//����һ�ֽڵ��Դ�������
extern uint8_t UI_CmpPrintfFlag;//���Դ��ڴ�ӡ���ݱ�־λ
//===========================================================================


//==============ҡ��ң����������================================================
#define huartBLErocker huart2 //��������ѡ��
#define USARTBLErocker USART2 //��������ѡ��
#define BLErockerLen 9 //�������ݰ�����

#define GPIOBLErockerStatus GPIOA//�������ڽ��ձ�־λ���͵�ƽ��ƣ�0����û����������
#define GPIO_PIN_BLErockerStatus GPIO_PIN_8//�������ڽ��ձ�־λ���͵�ƽ��ƣ�0����û����������
extern uint8_t BLErockerRxDa; //����һ�ֽ�������������
extern uint8_t BLErockerStatus;//�������ڽ��ձ�־λ���͵�ƽ��ƣ�0����û����������

//===========================================================================

//==============��е���Զ����������������======================================
#define huartBLEarm huart6 //��������ѡ��
#define USARTBLEarm USART6 //��������ѡ��
#define BLEarmLen 11 //�������ݰ�����

#define GPIOBLEarmStatus GPIOA//�������ڽ��ձ�־λ���͵�ƽ��ƣ�0����û����������
#define GPIO_PIN_BLEarmStatus GPIO_PIN_8//�������ڽ��ձ�־λ���͵�ƽ��ƣ�0����û����������
extern uint8_t BLEarmRxDa; //����һ�ֽ�������������
extern uint8_t BLEarmStatus;//�������ڽ��ձ�־λ���͵�ƽ��ƣ�0����û����������


extern uint8_t BLE_armTEMP[16];
//=============================================================================



extern uint16_t Test_PumpCmp;//��������ռ�ձ�
extern uint16_t Test_ValveCmp;//������ͨ��ռ�ձ�

#endif
