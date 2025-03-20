/***********************************************
���ƺ���ȫ��������
***********************************************/
#include "control.h"
//���״̬����
float x_pose=0, x_speed, angle_x, gyro_x, angle_z=0, gyro_z,last_angle=0;
float L_accel, R_accel, velocity_L, velocity_R;
//LQR״̬����ϵ��
float K1=-22.3606797749972, K2=-36.1268564861915, K3=-269.786509519640, K4=-27.7649997586234, K5=22.3606797749979, K6=4.57274492271650;
//Ŀ��״ֵ̬
float Target_x_speed=0, Target_angle_x=0, Target_gyro_z=0;
//�ٶȻ����PWMռ�ձȵı���ϵ��
float Ratio_accel=5948;			
//���ұ��������������
int Encoder_Left, Encoder_Right;
/**************************************************************************
Function: Control function
Input   : none
Output  : none
�������ܣ����еĿ��ƴ��붼��������
         5ms�ⲿ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��	
��ڲ�������
����  ֵ����				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//��ѹ������ر���
	static u8 Flag_Target;																//���ƺ�����ر������ṩ10ms��׼
	
	if(INT==0)		
	{   
		EXTI->PR=1<<12;                           					//����жϱ�־λ   
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//������̬��5msһ�Σ����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
		Encoder_Left=-Read_Encoder(3);            					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
		Encoder_Right=-Read_Encoder(4);           					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
		//��ȡ�ٶ�(m/s)��λ��(m)
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
		x_speed=(Encoder_Left+Encoder_Right)/2*PI*Diameter_67/1000/1560*Control_Frequency;
		x_pose+=x_speed/Control_Frequency;
		//��ȡ���(rad)�����ٶ�(rad/s)
		angle_x=Angle_Balance/180*PI;
		gyro_x=(angle_x-last_angle)*Control_Frequency;
		last_angle=angle_x;
		//��ȡת���ٶ�(rad/s)��ת���(rad)
		gyro_z=(Encoder_Right-Encoder_Left)/Wheel_spacing/1000*PI*Diameter_67/1000/1560*Control_Frequency;
		angle_z+=gyro_z/Control_Frequency;
		//Read_Distance();
		
		//�����������(LQR������)*/
		L_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x+K5*angle_z+K6*(gyro_z-Target_gyro_z));
		R_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x-K5*angle_z-K6*(gyro_z-Target_gyro_z));
		//�ٶȻ����PWMռ�ձ�
		velocity_L=(int)(Ratio_accel*(x_speed+L_accel/Control_Frequency));
		velocity_R=(int)(Ratio_accel*(x_speed+R_accel/Control_Frequency));
		//
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;  		//���������ṩ50ms�ľ�׼��ʱ��ʾ������Ҫ50ms�߾�����ʱ
		}
		if(Flag_Target==1)                        					//10ms����һ��
		{
			Voltage_Temp=Get_battery_volt();		    					//��ȡ��ص�ѹ		
			Voltage_Count++;                       						//ƽ��ֵ������
			Voltage_All+=Voltage_Temp;              					//��β����ۻ�
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ		
			return 0;	                                               
		}	
		Key();                                    					//ɨ�谴��״̬ ����˫�����Ըı�С������״̬
		Motor_Left=PWM_Limit(velocity_L,6900,-6900);
		Motor_Right=PWM_Limit(velocity_R,6900,-6900);				//PWM�޷�
		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//����Ƿ�С��������
			Flag_Stop=1;	                           					//���������͹رյ��
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//����Ƿ�С��������
			Flag_Stop=0;	                           					//��������¾��������
		//Choose(Encoder_Left,Encoder_Right);									//ת������ѡ��С��ģʽ
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//����������쳣
			Set_Pwm(Motor_Left,Motor_Right);         					//��ֵ��PWM�Ĵ���  
	 }       	
	 return 0;	  
} 

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left��Left wheel PWM��motor_right��Right wheel PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	    AIN1=1,			AIN2=0; //ǰ�� 
	else           			  AIN1=0,			AIN2=1; //����
	PWMA=myabs(motor_left);	
  if(motor_right>0)			BIN1=1,			BIN2=0;	//ǰ��
	else 	        			  BIN1=0,			BIN2=1; //����
	PWMB=myabs(motor_right);
	
}
/**************************************************************************
Function: PWM limiting range
Input   : IN��Input  max��Maximum value  min��Minimum value
Output  : Output
�������ܣ�����PWM��ֵ 
��ڲ�����IN���������  max���޷����ֵ  min���޷���Сֵ
����  ֵ���޷����ֵ
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(50); 
	if(tmp==1)
	{ 
		Flag_Stop=!Flag_Stop;
	}		

}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle��Car inclination��voltage��Voltage
Output  : 1��abnormal��0��normal
�������ܣ��쳣�رյ��		
��ڲ�����angle��С����ǣ�voltage����ѹ
����  ֵ��1���쳣  0������
**************************************************************************/	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000)//��ص�ѹ����10V�رյ��
	{	                                                 //��Ǵ���40�ȹرյ��
		temp=1;                                          //Flag_Stop��1�����������ƹرյ��
		AIN1=0;                                            
		AIN2=0;
		BIN1=0;
		BIN2=0;
	}
	else
		temp=0;
	return temp;			
}
	
/**************************************************************************
Function: Get angle
Input   : way��The algorithm of getting angle 1��DMP  2��kalman  3��Complementary filtering
Output  : none
�������ܣ���ȡ�Ƕ�	
��ڲ�����way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	if(way==1)                           //DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{	
		Read_DMP();                      	 //��ȡ���ٶȡ����ٶȡ����
		Angle_Balance=Pitch;             	 //����ƽ�����,ǰ��Ϊ��������Ϊ��
		Gyro_Balance=gyro[0];              //����ƽ����ٶ�,ǰ��Ϊ��������Ϊ��
		Gyro_Turn=gyro[2];                 //����ת����ٶ�
		Acceleration_Z=accel[2];           //����Z����ٶȼ�
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		if(Gyro_X>32768)  Gyro_X-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //��������ת��
		if(Accel_X>32768) Accel_X-=65536;                //��������ת��
		if(Accel_Y>32768) Accel_Y-=65536;                //��������ת��
		if(Accel_Z>32768) Accel_Z-=65536;                //��������ת��
		Gyro_Balance=-Gyro_X;                            //����ƽ����ٶ�
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //������ǣ�ת����λΪ��	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //������ǣ�ת����λΪ��
		accel_x=Accel_X/16384;                           //MPU6050��ʼ��Ϊ���ٶȼ�Ϊ��2g���õ���ԭʼ����Ϊ16λ���ݣ��������λΪ����λ��
		accel_y=Accel_Y/16384;                           //���Զ�ȡ��������λ��32768����Ӧ��2g������16384�������ݷֱ��ʣ�ԭʼ���ݳ���16384�õ�����Ϊm/S^2
		accel_z=Accel_Z/16384;
		Gyro_X=Gyro_X/16.4;                              //����������ת�������̡�2000��/s��Ӧ������16.4���õ�ԭʼ����Ϊ����32768����Ӧ��2000��/s
		Gyro_Y=Gyro_Y/16.4;                              //����32768/2000 = 16.4��Ҳ�ɲ鿴�ֲ�õ�������
		if(Way_Angle==2)		  	
		{
			 Pitch= -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//�������˲�����λΪ��
			 Roll = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//�����˲�
			 Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		Angle_Balance=Pitch;                              //����ƽ�����
		Gyro_Turn=Gyro_Z;                                 //����ת����ٶ�
		Acceleration_Z=Accel_Z;                           //����Z����ٶȼ�
	}

}
/**************************************************************************
Function: Absolute value function
Input   : a��Number to be converted
Output  : unsigned int
�������ܣ�����ֵ����
��ڲ�����a����Ҫ�������ֵ����
����  ֵ���޷�������
**************************************************************************/	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration��Z-axis acceleration��Angle��The angle of balance��encoder_left��Left encoder count��encoder_right��Right encoder count
Output  : 1��picked up  0��No action
�������ܣ����С���Ƿ�����
��ڲ�����Acceleration��z����ٶȣ�Angle��ƽ��ĽǶȣ�encoder_left���������������encoder_right���ұ���������
����  ֵ��1:С��������  0��С��δ������
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //��һ��
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //����1��С���ӽ���ֹ
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //����ڶ���
	 {
			if(++count1>200)       count1=0,flag=0;                       //��ʱ���ٵȴ�2000ms�����ص�һ��
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //����2��С������0�ȸ���������
			flag=2; 
	 } 
	 if(flag==2)                                                       //������
	 {
		  if(++count2>100)       count2=0,flag=0;                        //��ʱ���ٵȴ�1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //����3��С������̥��Ϊ�������ﵽ����ת��   
      {
				flag=0;                                                                                     
				return 1;                                                    //��⵽С��������
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance��Left encoder count��Right encoder count
Output  : 1��put down  0��No action
�������ܣ����С���Ƿ񱻷���
��ڲ�����ƽ��Ƕȣ���������������ұ���������
����  ֵ��1��С������   0��С��δ����
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                     //��ֹ���      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //����1��С������0�ȸ�����
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //��ʱ���ٵȴ� 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //����2��С������̥��δ�ϵ��ʱ����Ϊת��  
      {
				flag=0;
				flag=0;
				return 1;                         //��⵽С��������
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
�������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//���ת��  ת��=������������5msÿ�Σ�*��ȡƵ��/��Ƶ��/���ٱ�/����������
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
}



