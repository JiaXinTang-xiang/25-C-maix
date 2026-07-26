#include "imu_tempctrl_task.h"

/**
 * @brief  BMI088姿态解算（Mahony）
 * 通过TIM14每1ms一次中断实现1000hz的解算频率
 * @note 温控尚未实现（硬件没有）
 */
 
#define cheat TRUE  //作弊模式 去掉较小的gyro值
#define correct_Time_define 1000    //上电去0飘 1000次取平均
#define temp_times 300       //探测温度阈值

pid_type_def Temperature_PID;
float Temperature_PID_Para[3]={1000,10,40};
float H723_Temperature=0.f;
unsigned int adc_v;
float adcx;

extern float gyro[3], accel[3], temp; //陀螺仪原始值
float gyro_correct[3]={0};  //0飘初始值
float RefTemp = 40;   //Destination
extern float roll,pitch,yaw;//欧拉角
uint8_t attitude_flag=0;
uint32_t correct_times=0;

void IMU_Temperature_Ctrl();

void INS_init(void)
{
    PID_init(&Temperature_PID,PID_POSITION,Temperature_PID_Para,2000,200);  //加热
    Mahony_Init(1000); //mahony姿态解算初始化
//    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
    while (BMI088_init()) {
        RGB_Control(0,1,0);
    }
}

uint32_t temp_temperature = 0;
void IMU_Temperature_Ctrl() {
//    PID_calc(&Temperature_PID,temp,RefTemp); //温度pid  //需要调一下pid使得温度在40°左右
//    temp_temperature=(uint32_t)Temperature_PID.out;
//    if (Temperature_PID.out<0) {
//        temp_temperature=0;
//    }
//    htim3.Instance->CCR4 =  temp_temperature;
}

/**
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 任务参数
 * @retval: void
 * @details: IMU姿态控制任务函数
**/
static uint8_t first_mahony=0;
void INS_Task(void)
{
    static uint32_t count = 0;

    // ins updata
    if ((count% 1) == 0)
    {
        BMI088_read(gyro,accel,&temp);
        if (first_mahony == 0) //mahony上电快速初始化
        {
            first_mahony++;
            MahonyAHRSinit(accel[0],accel[1],accel[2],0,0,0);
        }
        if(attitude_flag==2)  //ekf的姿态解算
        {
            gyro[0]-=gyro_correct[0];   //减去陀螺仪0飘
            gyro[1]-=gyro_correct[1];
            gyro[2]-=gyro_correct[2];

            #if cheat              //作弊 可以让yaw很稳定 去掉比较小的值
                        if(fabsf(gyro[2])<0.003f)
                            gyro[2]=0;
            #endif

            //=================================================================================
            //mahony姿态解算部分
            Mahony_update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],0,0,0);
            Mahony_computeAngles(); //角度计算

            roll = getRoll();    // 滚转角
            pitch = getPitch();  // 俯仰角
            yaw = getYaw();      // 偏航角
        }
        else if(attitude_flag==1)   //状态1 开始1000次的陀螺仪0飘初始化
        {
            //gyro correct
            gyro_correct[0]+=gyro[0];
            gyro_correct[1]+=gyro[1];
            gyro_correct[2]+=gyro[2];
            correct_times++;
            if(correct_times>=correct_Time_define)
            {
                gyro_correct[0]/=correct_Time_define;
                gyro_correct[1]/=correct_Time_define;
                gyro_correct[2]/=correct_Time_define;
                attitude_flag=2; //go to 2 state
            }
        }
    }
    if ((count  % 10) == 0)
    {
//        // 100hz 的温度控制pid
//        IMU_Temperature_Ctrl();
//        //==================================================================================
//        //单片机内部温度测量
//        HAL_ADC_Start(&hadc3);
//        adc_v = HAL_ADC_GetValue(&hadc3);
//        adcx = (110.0-30.0)/(*(unsigned short*)(0x1FF1E840) - *(unsigned short*)(0x1FF1E820));
//        H723_Temperature = adcx*(adc_v - *(unsigned short*)(0x1FF1E820))+30;
//        //========================================================================================
//        static uint32_t temp_Ticks=0;
//        if((fabsf(temp-RefTemp)<0.5f)&&attitude_flag==0) //接近额定温度之差小于0.5° 开始计数
//        {
//            temp_Ticks++;
//            if(temp_Ticks>temp_times)   //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
//            {
                attitude_flag=1;  //go to correct state
//            }
//        }
    }
    count++;
}


/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {

    }
}
