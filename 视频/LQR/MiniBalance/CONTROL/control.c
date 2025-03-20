/***********************************************
控制函数全部在其下
***********************************************/
#include "control.h"
//相关状态数据
float x_pose=0, x_speed, angle_x, gyro_x, angle_z=0, gyro_z,last_angle=0;
float L_accel, R_accel, velocity_L, velocity_R;
//LQR状态反馈系数
float K1=-22.3606797749972, K2=-36.1268564861915, K3=-269.786509519640, K4=-27.7649997586234, K5=22.3606797749979, K6=4.57274492271650;
//目标状态值
float Target_x_speed=0, Target_angle_x=0, Target_gyro_z=0;
//速度换算成PWM占空比的比例系数
float Ratio_accel=5948;			
//左右编码器的脉冲计数
int Encoder_Left, Encoder_Right;
/**************************************************************************
Function: Control function
Input   : none
Output  : none
函数功能：所有的控制代码都在这里面
         5ms外部中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步	
入口参数：无
返回  值：无				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//电压测量相关变量
	static u8 Flag_Target;																//控制函数相关变量，提供10ms基准
	
	if(INT==0)		
	{   
		EXTI->PR=1<<12;                           					//清除中断标志位   
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
		Encoder_Left=-Read_Encoder(3);            					//读取左轮编码器的值，前进为正，后退为负
		Encoder_Right=-Read_Encoder(4);           					//读取右轮编码器的值，前进为正，后退为负
		//获取速度(m/s)、位移(m)
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
		x_speed=(Encoder_Left+Encoder_Right)/2*PI*Diameter_67/1000/1560*Control_Frequency;
		x_pose+=x_speed/Control_Frequency;
		//获取倾角(rad)、角速度(rad/s)
		angle_x=Angle_Balance/180*PI;
		gyro_x=(angle_x-last_angle)*Control_Frequency;
		last_angle=angle_x;
		//获取转向速度(rad/s)、转向角(rad)
		gyro_z=(Encoder_Right-Encoder_Left)/Wheel_spacing/1000*PI*Diameter_67/1000/1560*Control_Frequency;
		angle_z+=gyro_z/Control_Frequency;
		//Read_Distance();
		
		//计算输入变量(LQR控制器)*/
		L_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x+K5*angle_z+K6*(gyro_z-Target_gyro_z));
		R_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x-K5*angle_z-K6*(gyro_z-Target_gyro_z));
		//速度换算成PWM占空比
		velocity_L=(int)(Ratio_accel*(x_speed+L_accel/Control_Frequency));
		velocity_R=(int)(Ratio_accel*(x_speed+R_accel/Control_Frequency));
		//
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;  		//给主函数提供50ms的精准延时，示波器需要50ms高精度延时
		}
		if(Flag_Target==1)                        					//10ms控制一次
		{
			Voltage_Temp=Get_battery_volt();		    					//读取电池电压		
			Voltage_Count++;                       						//平均值计数器
			Voltage_All+=Voltage_Temp;              					//多次采样累积
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值		
			return 0;	                                               
		}	
		Key();                                    					//扫描按键状态 单击双击可以改变小车运行状态
		Motor_Left=PWM_Limit(velocity_L,6900,-6900);
		Motor_Right=PWM_Limit(velocity_R,6900,-6900);				//PWM限幅
		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被拿起
			Flag_Stop=1;	                           					//如果被拿起就关闭电机
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被放下
			Flag_Stop=0;	                           					//如果被放下就启动电机
		//Choose(Encoder_Left,Encoder_Right);									//转动右轮选择小车模式
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//如果不存在异常
			Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存器  
	 }       	
	 return 0;	  
} 

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left：Left wheel PWM；motor_right：Right wheel PWM
Output  : none
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	    AIN1=1,			AIN2=0; //前进 
	else           			  AIN1=0,			AIN2=1; //后退
	PWMA=myabs(motor_left);	
  if(motor_right>0)			BIN1=1,			BIN2=0;	//前进
	else 	        			  BIN1=0,			BIN2=1; //后退
	PWMB=myabs(motor_right);
	
}
/**************************************************************************
Function: PWM limiting range
Input   : IN：Input  max：Maximum value  min：Minimum value
Output  : Output
函数功能：限制PWM赋值 
入口参数：IN：输入参数  max：限幅最大值  min：限幅最小值
返回  值：限幅后的值
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
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
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
Input   : angle：Car inclination；voltage：Voltage
Output  : 1：abnormal；0：normal
函数功能：异常关闭电机		
入口参数：angle：小车倾角；voltage：电压
返回  值：1：异常  0：正常
**************************************************************************/	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000)//电池电压低于10V关闭电机
	{	                                                 //倾角大于40度关闭电机
		temp=1;                                          //Flag_Stop置1，即单击控制关闭电机
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
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度	
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
		Read_DMP();                      	 //读取加速度、角速度、倾角
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn=gyro[2];                 //更新转向角速度
		Acceleration_Z=accel[2];           //更新Z轴加速度计
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //计算倾角，转换单位为度	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //计算倾角，转换单位为度
		accel_x=Accel_X/16384;                           //MPU6050初始化为加速度计为±2g，得到的原始数据为16位数据，并且最高位为符号位，
		accel_y=Accel_Y/16384;                           //所以读取到的数据位±32768，对应着2g，所以16384就是数据分辨率，原始数据除以16384得到数据为m/S^2
		accel_z=Accel_Z/16384;
		Gyro_X=Gyro_X/16.4;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，得到原始数据为整幅32768，对应±2000°/s
		Gyro_Y=Gyro_Y/16.4;                              //所以32768/2000 = 16.4，也可查看手册得到该数据
		if(Way_Angle==2)		  	
		{
			 Pitch= -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//卡尔曼滤波，单位为度
			 Roll = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//互补滤波
			 Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计
	}

}
/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
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
Input   : Acceleration：Z-axis acceleration；Angle：The angle of balance；encoder_left：Left encoder count；encoder_right：Right encoder count
Output  : 1：picked up  0：No action
函数功能：检测小车是否被拿起
入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计数
返回  值：1:小车被拿起  0：小车未被拿起
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //第一步
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //条件1，小车接近静止
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //进入第二步
	 {
			if(++count1>200)       count1=0,flag=0;                       //超时不再等待2000ms，返回第一步
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //条件2，小车是在0度附近被拿起
			flag=2; 
	 } 
	 if(flag==2)                                                       //第三步
	 {
		  if(++count2>100)       count2=0,flag=0;                        //超时不再等待1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //条件3，小车的轮胎因为正反馈达到最大的转速   
      {
				flag=0;                                                                                     
				return 1;                                                    //检测到小车被拿起
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下
入口参数：平衡角度；左编码器读数；右编码器读数
返回  值：1：小车放下   0：小车未放下
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                     //防止误检      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //条件1，小车是在0度附近的
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				flag=0;
				return 1;                         //检测到小车被放下
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
函数功能：编码器读数转换为速度（mm/s）
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//电机转速  转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//求出编码器速度=转速*周长
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//求出编码器速度=转速*周长
}



