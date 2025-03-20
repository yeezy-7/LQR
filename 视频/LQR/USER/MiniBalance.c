//主函数
#include "stm32f10x.h"
#include "sys.h"
u8 Way_Angle=3;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; //相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //电机停止标志位和显示标志位  默认停止 显示打开
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //温度变量
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
u32 Distance;                               //超声波测距
u8 delay_50,delay_flag,PID_Send; 						//延时和调参相关变量
float Acceleration_Z;                       //Z轴加速度计  
float Balance_Kp=22500,Balance_Kd=108,Velocity_Kp=16000,Velocity_Ki=80,Turn_Kp=4200,Turn_Kd=50;//PID参数（放大100倍）
int main(void)
{ 
  MY_NVIC_PriorityGroupConfig(2);	//设置中断分组
	delay_init();	    	            //延时函数初始化	
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试
	LED_Init();                     //初始化与 LED 连接的硬件接口
	KEY_Init();                     //按键初始化
	MiniBalance_PWM_Init(7199,0);   //初始化PWM 10KHZ与电机硬件接口，用于驱动电机
	uart_init(115200);	            //串口1初始化
	uart3_init(9600);             	//串口3初始化，用于蓝牙模块
	Encoder_Init_TIM3();            //编码器接口
	Encoder_Init_TIM4();            //初始化编码器4
	Adc_Init();                     //adc初始化
	IIC_Init();                     //IIC初始化
	OLED_Init();                    //OLED初始化	    
	MPU6050_initialize();           //MPU6050初始化	
	DMP_Init();                     //初始化DMP 
	//TIM2_Cap_Init(0XFFFF,72-1);	    //超声波初始化
	MiniBalance_EXTI_Init();        //MPU6050 5ms定时中断初始化，节省定时器资源，减少cpu负担
	while(1)
	{
		if(Flag_Show==0)          		//使用OLED显示屏
		{
			
			 oled_show();          			//显示屏打开
		}
		
		delay_flag=1;	
		delay_50=0;
		while(delay_flag);	     			//示波器需要50ms	高精度延时，delay函数不满足要求，故使用MPU6050中断提供50ms延时
	}
}

