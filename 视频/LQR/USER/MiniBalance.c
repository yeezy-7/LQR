//������
#include "stm32f10x.h"
#include "sys.h"
u8 Way_Angle=3;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; //��صı���
u8 Flag_Stop=1,Flag_Show=0;                 //���ֹͣ��־λ����ʾ��־λ  Ĭ��ֹͣ ��ʾ��
int Motor_Left,Motor_Right;                 //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //�¶ȱ���
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
u32 Distance;                               //���������
u8 delay_50,delay_flag,PID_Send; 						//��ʱ�͵�����ر���
float Acceleration_Z;                       //Z����ٶȼ�  
float Balance_Kp=22500,Balance_Kd=108,Velocity_Kp=16000,Velocity_Ki=80,Turn_Kp=4200,Turn_Kd=50;//PID�������Ŵ�100����
int main(void)
{ 
  MY_NVIC_PriorityGroupConfig(2);	//�����жϷ���
	delay_init();	    	            //��ʱ������ʼ��	
	JTAG_Set(JTAG_SWD_DISABLE);     //�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //��SWD�ӿ� �������������SWD�ӿڵ���
	LED_Init();                     //��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //������ʼ��
	MiniBalance_PWM_Init(7199,0);   //��ʼ��PWM 10KHZ����Ӳ���ӿڣ������������
	uart_init(115200);	            //����1��ʼ��
	uart3_init(9600);             	//����3��ʼ������������ģ��
	Encoder_Init_TIM3();            //�������ӿ�
	Encoder_Init_TIM4();            //��ʼ��������4
	Adc_Init();                     //adc��ʼ��
	IIC_Init();                     //IIC��ʼ��
	OLED_Init();                    //OLED��ʼ��	    
	MPU6050_initialize();           //MPU6050��ʼ��	
	DMP_Init();                     //��ʼ��DMP 
	//TIM2_Cap_Init(0XFFFF,72-1);	    //��������ʼ��
	MiniBalance_EXTI_Init();        //MPU6050 5ms��ʱ�жϳ�ʼ������ʡ��ʱ����Դ������cpu����
	while(1)
	{
		if(Flag_Show==0)          		//ʹ��OLED��ʾ��
		{
			
			 oled_show();          			//��ʾ����
		}
		
		delay_flag=1;	
		delay_50=0;
		while(delay_flag);	     			//ʾ������Ҫ50ms	�߾�����ʱ��delay����������Ҫ�󣬹�ʹ��MPU6050�ж��ṩ50ms��ʱ
	}
}

