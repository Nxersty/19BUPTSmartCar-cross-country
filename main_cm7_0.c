/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "SoundLocation_v1.h"


// 宏定义-----------------------------------------------------------------------
#define ADC_TO_V_F 0.000805664 // ADC值至电压(V)转换系数(3.3 / 4096)
#define DEBOUNCE_TIME 300      // 按键消抖时间(ms)
#define V_TEST_TO_REAL_F 4     // 电压检测端口电压至实际电压转换系数
#define CHARGE_THRESHOLD 12    // 电容组充电电压值(V)
#define INIT_SPEED 3000        // 电机初始速度(占空比20%)
#define TURNING_SPEED 2000     // 转弯时单轮速度(占空比20%)
#define TURN_TIME_MAX 6000     // 转弯最大时间(ms)
#define BRAKE_TIME 500         // 转弯后刹车时间(ms)
#define SPEED_TIMEOUT 1000     // 测速时间限制(ms)(超过这个时间还没有转过1/6圈视为速度为0)
#define CTRL_INTERVAL 30       // 控制代码运行时间间隔(ms)
#define TURN_TARGET_F 0.8      // 转向目标角度系数
#define TURN_SPEED 0.2         // 转弯动力轮速度(m/s)
#define TURN_STEP 20           // 转弯动力轮步进值(占空比)
#define TURN_INIT_SPEED 1500   // 转弯动力轮初始值(占空比)

// 变量类型定义------------------------------------------------------------------
typedef enum mode_t{
    Idle,Charging,RunningStraight,Turning
} mode_t;

typedef enum debugMode_t{
    Normal,NormalDebug,StraightDeubug,TurnDebug
} debugMode_t;



// 全局变量---------------------------------------------------------------------    
mode_t Mode = Idle;     //表示当前车辆的运行模式
debugMode_t DebugMode = Normal;     //表示当前的调试模式
direction_t Direction = Forward;    //用于表示车辆的行进方向，可能是 Forward（前进）或其他类似的方向
sint32 SpeedL = 0;  //分别表示左右轮的速度
sint32 SpeedR = 0;

signed short mpu6050_gyro_x , mpu6050_gyro_y , mpu6050_gyro_z = 0;    //分别表示车辆的三轴陀螺仪数据。
signed short mpu6050_acc_x , mpu6050_acc_y , mpu6050_acc_z = 0;    //分别表示车辆的三轴加速度计数据
double GzDegPerSec = 0;     //表示陀螺仪的角速度和角度
double GzDeg = 0;
double AxMPerSqrSec = 0;    //表示加速度计的加速度
double AxMPerSec = 0;

signed short Gx , Gy , Gz = 0;
signed short Ax , Ay , Az = 0;
double GzDegPerSec = 0;
double GzDeg = 0;
double AxMPerSec = 0;
double AxMPerSec = 0;

int DebugDeg = 8;   //用于调试的角度和方向
int DebugDir = BackRight;

float StraightSpeed = 1; // 直行速度(m/s)
float StraightGzTolerance = 100; // 直行角速度容差绝对值

sint32 TurnSpeedL[9] = {0, 0, 2500, 2300, 2000, 0, 0, 0, 0};    //分别表示左轮和右轮在不同转弯情况下的速度
sint32 TurnSpeedR[9] = {0, 0, 0, 0, 0, 2000, 2300, 2500, 0};

long LastTurnTick = 0;  //记录上一次转弯和陀螺仪数据更新的时间
long LastMPUTick = 0;

volatile long TickMillis = 0;

volatile long LastSpeedTickL = 0;   //记录左右轮速度的上一次更新时间
volatile long LastSpeedTickR = 0;

volatile float SpeedLReal = 0;  //记录实际的左右轮速度
volatile float SpeedRReal = 0;

long LastCtrlTick = 0;  //记录上一次控制操作的时间和控制操作的时间间隔
long CtrlInterval = 0;

char txt[128] = {0};

// **************************** 代码区域 ****************************

int core0_main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化
    
    // 此处编写用户代码 例如外设初始化代码等
    
    //关闭CPU总中断
    
    //关闭看门狗，如果不设置看门狗喂狗需要关闭
    
    //读取总线频率
    
    gpio_init(P19_0, GPO, GPIO_LOW, GPO_PUSH_PULL);              // 初始化 LED1 输出 默认高电平 推挽输出模式

    gpio_init(P20_0, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(P20_1, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(P20_2, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(P20_3, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY4 输入 默认高电平 上拉输入

    gpio_init(P21_5, GPI, GPIO_HIGH, GPI_PULL_UP);            // 初始化 SWITCH1 输入 默认高电平 上拉输入
    gpio_init(P21_6, GPI, GPIO_HIGH, GPI_PULL_UP);            // 初始化 SWITCH2 输入 默认高电平 上拉输入
    
    uart_sbus_init(UART_4,115200,UART4_RX_P14_0,UART4_TX_P14_1);    //串口P14.0管脚输出，P14.1输入，波特率115200
    //soft_iic_init()    通信线初始化
    while(mpu6050_init());

    motor_information_out_init();
    motor_speed_curve_init();

    adc_init(ADC_CHANNEL1, ADC_12BIT);                                          
    adc_init(ADC_CHANNEL2, ADC_12BIT);                                          
    adc_init(ADC_CHANNEL3, ADC_10BIT);                                          
    adc_init(ADC_CHANNEL4, ADC_8BIT);                                           

    adc_init(ADC_CHANNEL5, ADC_12BIT);                                          
    adc_init(ADC_CHANNEL6, ADC_10BIT);                                         
    adc_init(ADC_CHANNEL7, ADC_8BIT);       S                                    
    adc_init(ADC_CHANNEL8, ADC_8BIT); 
    
    
    //配置外部中断，用于处理外部触发的时间
    //配置CPU总中断
    //通知CPU1,CPU0初始化完成

//主循环-------------------------------------------------------------
    while(true)
    {
        getDirection();     //获取方向信息
        switchMode();    //切换模式
        readMPU6050();    //读取MPU6050传感器数据
        testStopped();    //测试是否停车
        CtrlInterval = TickMillis - LastCtrlTick;       //计算控制间隔时间
        if (CtrlInterval >= CTRL_INTERVAL) 
        {
            switch (Mode) {
                case Idle:
                    SpeedL = 0;
                    SpeedR = 0; // 停车
                    changeParam();
                    break;
                case Charging:
                    testCharge();
                    break;
                case RunningStraight:
                    runStraight();
                    break;
                case Turning:
                    turn();
                    break;
                default:
                    break;
            }
            LastCtrlTick = TickMillis;
        }

        MotorCtrl(SpeedL, SpeedR);
    }

}

// **************************** 函数区域 ****************************

//LED灯闪烁times次
void blink(unsigned char times) 
{
    int c;
    for(c = 0; c < 2*times;c++)
    {
        gpio_set_level(P19_0, 1)
    }
}


//切换模式
void switchMode()
{
    DebugMode = ((P21_5) << 1) + gpio_get_level(P21_6);     //通过读取开关信号设置调试模式(DebugMode)
    switch (DebugMode)
    {
        case NormalDebug:
        case StraightDeubug:
            if(key_get_state(KEY_1))
            {
                if(Mode == Idle)
                {
                    sprintf(txt, "Running\n");
                    uart_write_string(UART_0,txt);
                    Mode = RunningStraight;
                    SpeedL = INIT_SPEED;
                    SpeedR = INIT_SPEED;
                    GzDeg = 0;
                    AxMPerSec = 0;
                    ADCCount = 0;
                }
                else{
                    sprintf(txt, "Idle\n");
                    uart_write_string(UART_0,txt);
                    Mode = Idle;
                }
                system_delay_ms(DEBOUNCE_TIME);
            }
            break;
        case Normal:
            if(Mode == Idle)
            {
                Mode = Charging;
            }
            break;
        case TurnDebug:
            if(key_get_state(KEY_1))
            {
                if(Mode == Idle)
                {
                    sprintf(txt, "Turning\n");
                    uart_write_string(UART_0,txt);
                    Mode = Turning;
                    SpeedL = 0;
                    SpeedR = 0;
                    Motor();
                    GzDeg = 0;
                }
                else{
                    sprintf(txt, "Idle\n");
                    uart_write_string(UART_0,txt);
                    Mode = Idle;
                }
                system_delay_ms(DEBOUNCE_TIME);
            }
            break;
        default:
            break;
    }

    if(DebugMode == Normal || DebugMode == NormalDebug)
    {
        if (Direction == Forward || Direction == FrontRight
            || Direction == FrontLeft) {
            if (Mode == Turning) 
            {
                //Motor();
                SpeedL = 0;
                SpeedR = 0;
                AxMPerSec = 0;
                SpeedLReal = 0;
                SpeedRReal = 0;
                Mode = RunningStraight;
                Direction = Forward;
                ErrorInc.lLast = 0;
                ErrorInc.lLastD = 0;
                ErrorInc.rLast = 0;
                ErrorInc.rLastD = 0;
                GzDegPerSec = 0;
                GzDeg = 0;
                AxMPerSec = 0;
                if (DebugMode == TurnDebug) {
                    Mode = Idle;
                }
                system_delay_ms(BRAKE_TIME);
                SpeedL = INIT_SPEED;
                SpeedR = INIT_SPEED;
            }
        }
        else if (Mode == RunningStraight) { 
            
            Mode = Turning;

            LastTurnTick = TickMillis;

            ErrorLoc.lLast = 0;
            ErrorLoc.rLast = 0;
            ErrorLoc.lTotal = 0;
            ErrorLoc.rTotal = 0;

            if (Dir[Direction] > 1) {
                SpeedL = 0;
                SpeedR = TURN_INIT_SPEED;
            }
            else if (Dir[Direction] < -1) {
                SpeedR = 0;
                SpeedL = TURN_INIT_SPEED;
            }

            //Motor();
            GzDeg = 0;
            AxMPerSec = 0;
        }
    }
}

//检测电量是否充满，充满则发车
void testCharge()
{
    //float chargeV = adc_convert(ADC);
    chargeV *= ADC_TO_V_F * V_TEST_TO_REAL_F;
    if (chargeV >= CHARGE_THRESHOLD) {
        Mode = RunningStraight;
        SpeedL = INIT_SPEED;
        SpeedR = INIT_SPEED;
        Motor(SpeedL, SpeedR);
        system_delay_ms(2000);
    }
#ifdef DEBUG
    sprintf(txt, "chargeV = %f\n", chargeV);
    uart_write_string(UART_0,txt);
    system_delay_ms(200);
#endif
}

//走直线
void runStraight()
{
    getPIDIncResult(&SpeedL, &SpeedR, StraightSpeed - SpeedLReal,StraightSpeed - SpeedRReal);
    if (GzDegPerSec > StraightGzTolerance || GzDegPerSec < -StraightGzTolerance) {
        SpeedL = 0;
        SpeedR = 0;
    }

#ifdef DEBUG
    sprintf(txt, "Running straight:\n");
    uart_write_string(UART_0,txt);
    sprintf(txt, "SpeedLReal = %f    SpeedRReal = %f\n", SpeedLReal,
            SpeedRReal);
    uart_write_string(UART_0,txt);
    sprintf(txt, "SpeedL = %ld    SpeedR = %ld\n", SpeedL, SpeedR);
    uart_write_string(UART_0,txt);
    sprintf(txt, "GzDegPerSec = %f\n", GzDegPerSec);
    Uuart_write_string(UART_0,txt);
#endif  
}

//转向
void turn()
{
    unsigned char stopTurning = 0;

    if (DebugMode == TurnDebug) {
        Direction = DebugDir;
    }
    if (Dir[Direction] > 1) {
    // 如果当前车头方向尚未到达目标方向，并且转弯时间未超过设定的最大时间
        if (GzDeg < Dir[Direction] * 22.5 && TickMillis - LastTurnTick < TURN_TIME_MAX) {
            SpeedL = 0;  // 左轮速度设为0
            // 根据实际右轮速度调整右轮速度
            if (SpeedRReal < TURN_SPEED) {
                SpeedR += TURN_STEP;
            } else if (SpeedRReal > TURN_SPEED) {
                SpeedR -= TURN_STEP;
            }
        } else {
            stopTurning = 1;  // 停止转弯
        }
    }
    // 如果目标方向小于-1，表示需要左转
    else if (Dir[Direction] < -1) {
        // 如果当前车头方向尚未到达目标方向，并且转弯时间未超过设定的最大时间
        if (GzDeg > Dir[Direction] * 22.5 && TickMillis - LastTurnTick < TURN_TIME_MAX) {
            SpeedR = 0;  // 右轮速度设为0
            // 根据实际左轮速度调整左轮速度
            if (SpeedLReal < TURN_SPEED) {
                SpeedL += TURN_STEP;
            } else if (SpeedLReal > TURN_SPEED) {
                SpeedL -= TURN_STEP;
            }
        } else {
            stopTurning = 1;  // 停止转弯
        }
    }
    // 如果目标方向在[-1, 1]之间，停止转弯
    else {
        stopTurning = 1;
    }

    // 如果需要停止转弯
    if (stopTurning) {
        // 停止电机控制，重置速度和相关参数
        //MotorCtrl(0, 0);
        SpeedL = 0;
        SpeedR = 0;
        AxMPerSec = 0;
        SpeedLReal = 0;
        SpeedRReal = 0;
        Mode = RunningStraight;
        Direction = Forward;
        //ADCCount = 0;
        ErrorInc.lLast = 0;
        ErrorInc.lLastD = 0;
        ErrorInc.rLast = 0;
        ErrorInc.rLastD = 0;
        GzDegPerSec = 0;
        GzDeg = 0;
        AxMPerSec = 0;
        if (DebugMode == TurnDebug) {
            Mode = Idle;
        }
        delayms(BRAKE_TIME);  // 停车后的延时
        SpeedL = INIT_SPEED;  // 重新设置速度
        SpeedR = INIT_SPEED;
    }
}

//找信标位置
void getDirection() 
{
    if (ADCCount >= XCORR_SIZE) {
        Direction = locateSound((sint16*)s1, (sint16*)s2, (sint16*)s3,(sint16*)s4);
        //ADCCount = 0;

#ifdef DEBUG
        sprintf(txt, "direction = %d\n", Direction);
        UART_PutStr(UART0, txt);
#endif

    }

#ifdef DEBUG
    sprintf(txt, "ADCCount = %d\n", ADCCount);
    UART_PutStr(UART0, txt);
#endif
}

//读取MPU6050加速度计和陀螺仪数据
void readMPU6050()
{
    if(LastMPUTick != TickMillis)
    {
        int dt;
        mpu6050_get_acc();
        mpu6050_get_gyro();
        dt = TickMillis - LastMPUTick;
        LastMPUTick = TickMillis;
        AxMPerSqrSec = -mpu6050_acc_x * 9.8 / 16384;
        AxMPerSec += AxMPerSqrSec / 1000 * dt;
        GzDegPerSec = mpu6050_gyro_z / 65.5;
        GzDeg += GzDegPerSec / 1000 * dt;
    }
}

//检测速度是否为0
void testStopped()
{
    if (TickMillis - LastSpeedTickL > SPEED_TIMEOUT) {
        SpeedLReal = 0;
    }
    if (TickMillis - LastSpeedTickR > SPEED_TIMEOUT) {
        SpeedRReal = 0;
    }
}