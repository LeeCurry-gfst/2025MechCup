/***********
*�����ִ���ΪMPU6050��صײ����ʵ��
*�Ѿ���ֲDMP�⣬�ܹ������̬����
************/


#include "stm32f4xx.h"                  // Device header
#include "i2c.h"
#include "MPU6050_reg.h"
#include "MPU6050.h"
#include "..\MPU6050\inv_mpu.h"
#include "..\MPU6050\inv_mpu_dmp_motion_driver.h"
#include "math.h"

#define p30 1073741824.0f


/************************************MPU6050��������ʵ��**Ӳ��IIC��ȡ����*****************************************/
/**
  * ��    ����MPU6050д�Ĵ���
  * ��    ����RegAddress �Ĵ�����ַ����Χ���ο�MPU6050�ֲ�ļĴ�������
  * ��    ����Data Ҫд��Ĵ��������ݣ���Χ��0x00~0xFF
  * �� �� ֵ����
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	
	HAL_I2C_Mem_Write(&hi2c1 ,MPU6050_ADDRESS,RegAddress,I2C_MEMADD_SIZE_8BIT ,&Data,1,1000);							
}

/**
  * ��    ����MPU6050���Ĵ���
  * ��    ����RegAddress �Ĵ�����ַ����Χ���ο�MPU6050�ֲ�ļĴ�������
  * �� �� ֵ����ȡ�Ĵ��������ݣ���Χ��0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	HAL_I2C_Mem_Read(&hi2c1 ,MPU6050_ADDRESS,RegAddress,I2C_MEMADD_SIZE_8BIT ,&Data,1,1000);			
	
	return Data;
}

void MPU6050_Init(void)
{
	/*MPU6050�Ĵ�����ʼ������Ҫ����MPU6050�ֲ�ļĴ����������ã��˴��������˲�����Ҫ�ļĴ���*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);				//��Դ����Ĵ���1��ȡ��˯��ģʽ�������¶ȴ�������ѡ��ʱ��ԴΪX��������
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);				//��Դ����Ĵ���2������Ĭ��ֵ0���������������
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//�����ʷ�Ƶ�Ĵ��������ò�����
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);					//���üĴ���������DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);			//���������üĴ�����ѡ��������Ϊ��2000��/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);			//���ٶȼ����üĴ�����ѡ��������Ϊ��16g
}

/**
  * ��    ����MPU6050��ȡID��
  * ��    ������
  * �� �� ֵ��MPU6050��ID��
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//����WHO_AM_I�Ĵ�����ֵ
}

/**
  * ��    ����MPU6050��ȡ����
  * ��    ����AccX AccY AccZ ���ٶȼ�X��Y��Z������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
  * ��    ����GyroX GyroY GyroZ ������X��Y��Z������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
  * ��    ����Temp �¶ȴ����������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
  * �� �� ֵ����
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, int16_t *Temp)
{
	uint8_t DataH, DataL;								//�������ݸ�8λ�͵�8λ�ı���
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//��ȡ���ٶȼ�X��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//��ȡ���ٶȼ�X��ĵ�8λ����
	*AccX = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//��ȡ���ٶȼ�Y��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//��ȡ���ٶȼ�Y��ĵ�8λ����
	*AccY = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//��ȡ���ٶȼ�Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//��ȡ���ٶȼ�Z��ĵ�8λ����
	*AccZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//��ȡ������X��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//��ȡ������X��ĵ�8λ����
	*GyroX = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//��ȡ������Y��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//��ȡ������Y��ĵ�8λ����
	*GyroY = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//��ȡ������Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//��ȡ������Z��ĵ�8λ����
	*GyroZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);		//��ȡ�¶ȴ������ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);		//��ȡ�¶ȴ������ĵ�8λ����
	*Temp = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	
}
/************************************MPU6050��������ʵ��**Ӳ��IIC��ȡ����*****************************************/


/**********************************DMP����ֲ���ʼ��************************************************************************/
//MPU6050�Բ���
//����ֵ:0,����
//    ����,ʧ��
uint8_t run_self_test(void)
{
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}else return result;
}
/*���������Ƿ�����ؼ���*/
unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;
	if(row[0]>0) b=0;
	else if(row[0]<0) b=4;
	else if(row[1]>0) b=1;
	else if(row[1]<0) b=5;
	else if(row[2]>0) b=2;
	else if(row[2]<0) b=6;
	else b=7;
	
	return b;
}

/*���������Ƿ���*/
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx+3)<<3;
	scalar |= inv_row_2_scale(mtx+6)<<6;
	
	return scalar ;
}


//mpu6050,dmp��ʼ��
//����ֵ:0,����
//    ����,ʧ��
uint8_t  MPU_DMP_Init(void)
{
	uint8_t  res=0;
	struct int_param_s int_param;//���ûʲô�ã�����Ϊ���ܸ���ʵ�ε�������
	
	static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

	
	if(mpu_init(&int_param)==0)	//��ʼ��MPU6050
	{	 
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//��������Ҫ�Ĵ�����
		if(res)return 1; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//����FIFO
		if(res)return 2; 
		res=mpu_set_sample_rate(100);	//���ò�����
		if(res)return 3; 
		res=dmp_load_motion_driver_firmware();		//����dmp�̼�
		if(res)return 4; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//���������Ƿ���
		if(res)return 5; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//����dmp����
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		res=dmp_set_fifo_rate(100);	//����DMP�������(��󲻳���200Hz)
		if(res)return 7;   
		res=run_self_test();		//�Լ�
		if(res)return 8;    
		res=mpu_set_dmp_state(1);	//ʹ��DMP
		if(res)return 9;     
	}
	return 0;
}
//�õ�dmp����������(ע��,��������Ҫ�Ƚ϶��ջ,�ֲ������е��)
//pitch:������ ����:0.1��   ��Χ:-90.0�� <---> +90.0��
//roll:�����  ����:0.1��   ��Χ:-180.0��<---> +180.0��
//yaw:�����   ����:0.1��   ��Χ:-180.0��<---> +180.0��
uint8_t  MPU_DMP_GetData(float *pitch,float *roll,float *yaw)
{
	float fquat[4]={0.0f,0.0f,0.0f,0.0f};
	short gyro[3];
	short accel[3]; 
	long quat[4];
    unsigned long timestamp; 
	short sensors; 
	unsigned char more;
	if(dmp_read_fifo (gyro,accel,quat,&timestamp,&sensors ,&more))//����
	{
		return 1;
	}
	if(sensors &INV_WXYZ_QUAT ) //�з���ֵ
	{
		for(int i=0;i<4; i++)
		{
			fquat [i]= quat [i]/p30;
		}
		
		*pitch = asin (-2*fquat [1]*fquat[3]+2*fquat[0]*fquat[2])*57.3;
		*roll = atan2 (2*fquat[2]*fquat[3]+2*fquat[0]*fquat[1],-2*fquat[1]*fquat[1]-2*fquat[2]*fquat[2]+1)*57.3;
		*yaw = atan2(2*(fquat[1]*fquat[2]+fquat[0]*fquat[3]),fquat[0]*fquat[0]+fquat[1]*fquat[1]-fquat[2]*fquat[2]-fquat[3]*fquat[3])*57.3;
	}
	return 0;
}

/**********************************DMP����ֲ���ʼ��************************************************************************/

