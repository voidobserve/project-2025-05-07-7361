/******************************************************************************
;  *       @型号                 : MC32F7361
;  *       @创建日期             : 2021.12.21
;  *       @公司/作者            : SINOMCU-FAE
;  *       @晟矽微技术支持       : 2048615934
;  *       @晟矽微官网           : http://www.sinomcu.com/
;  *       @版权                 : 2021 SINOMCU公司版权所有.
;  *----------------------摘要描述---------------------------------

******************************************************************************/

#include "user.h"

/************************************************
;  *    @函数名          : CLR_RAM
;  *    @说明            : 清RAM
;  *    @输入参数        :
;  *    @返回参数        :
;  ***********************************************/
void CLR_RAM(void)
{
    for (FSR0 = 0; FSR0 < 0xff; FSR0++)
    {
        INDF0 = 0x00;
    }

    FSR0 = 0xFF;
    INDF0 = 0x00;
}

/************************************************
;  *    @函数名            : IO_Init
;  *    @说明              : IO初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void IO_Init(void)
{
    IOP0 = 0x00;   // io口数据位
    OEP0 = 0x3F;   // io口方向 1:out  0:in
    PUP0 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP0 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P0ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP1 = 0x00;   // io口数据位
    OEP1 = 0xFF;   // io口方向 1:out  0:in
    PUP1 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP1 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P1ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP2 = 0x00; // io口数据位
    OEP2 = 0x0F; // io口方向 1:out  0:in
    PUP2 = 0x00; // io口上拉电阻   1:enable  0:disable
    PDP2 = 0x00; // io口下拉电阻   1:enable  0:disablea

    PMOD = 0x00;  // P00、P01、P13 io端口值从寄存器读，推挽输出
    DRVCR = 0x80; // 普通驱动
}

void timer1_config(void)
{
    T1CR |= (0x01 << 7) | (0x01 << 1); // 使能定时器，时钟源选择CPU, 4分频
    T1LOAD = 109 - 1;                  // 100us (从sdk复制过来的会有些误差，这里做了补偿)
    // T1EN = 1;
    // T1IE = 1;
}

// 只使用 NPWM2
void timer2_pwm_config(void)
{
    // T2CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 1) | (0x01 << 0); // 使能定时器，使能PWM, CPU, 8分频
    T2CR = (0x01 << 7) | (0x01 << 6); // 使能定时器，使能PWM, CPU,
    T2LOAD = 215 - 1;                 //
    T2DATA = 215 - 38;                /* 由于只使用 NPWM 互补PWM，这里占空比计算要用 周期计数值 减去 比较值，才是互补PWM的占空比 */
    // PWMCR3 = DEF_SET_BIT6 | DEF_SET_BIT5 | DEF_SET_BIT4; // 使能FPWM,NPWM,正向输出
    PWMCR3 |= (0x01 << 5); // 使能NPWM2输出波形
    // T2EN = 1;
}

void timer2_pwm_set_feq(void)
{
    PWMCR1 = (0x01 << 4); // FTMR 选择 FHOSC

    T2CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 3); // 使能定时器，使能PWM, 时钟源选择FTMR，时钟源不分频
    T2LOAD = 255 - 1;                               //
    T2DATA = 255 - 50;                              /* 由于只使用 NPWM 互补PWM，这里占空比计算要用 周期计数值 减去 比较值，才是互补PWM的占空比 */
}

void adc_config(void)
{
    // 检测充电座放电电流的引脚：
    // P15OE = 0; // 输入模式
    // P15DC = 1; // 模拟模式

    // 检测充电电压的引脚，充电电压 1/10分压
    P12OE = 0; // 输入模式
    P12DC = 1; // 模拟模式

    // 检测电池电压的引脚，电池电压 1/2分压
    P05OE = 0; // 输入模式
    P05DC = 1; // 模拟模式

    ADCR0 = 0x0B; // 12位精度，使能adc
    ADCR1 = 0x80; // adc转换时钟选择 FHIRC/32，使用內部2.0V参考电压
    ADCR2 = 0x0F; // 采样时间，只能固定是15 个 ADCLK
}

void adc_sel_pin(u8 adc_pin)
{

    // P12 AN7
    if (ADC_PIN_DETECT_CHARGE == adc_pin)
    {
        ADCHS3 = 0;
        ADCHS2 = 1;
        ADCHS1 = 1;
        ADCHS0 = 1;
    }
    else if (ADC_PIN_DETECT_BATTERY == adc_pin)
    {
        // P05 AN4
        ADCHS3 = 0;
        ADCHS2 = 1;
        ADCHS1 = 0;
        ADCHS0 = 0;
    }

    delay_ms(1); // 等待adc稳定
}

// adc采集+滤波
u16 adc_getval(void)
{
    u16 adc_val_tmp = 0;     // 存放单次采集到的ad值
    u32 adc_val_sum = 0;     // 存放所有采集到的ad值的累加
    u16 get_adcmax = 0;      // 存放采集到的最大的ad值
    u16 get_adcmin = 0xFFFF; // 存放采集到的最小的ad值(初始值为最大值)
    u8 i = 0;
    for (i = 0; i < 20; i++)
    {
        ADEOC = 0; // 写0开始转换
        while (!ADEOC)
            ; // 等待转换完成
        adc_val_tmp = ADRH << 4 | (ADRL & 0x0F);

        if (i < 2)
            continue; // 丢弃前两次采样的
        if (adc_val_tmp > get_adcmax)
            get_adcmax = adc_val_tmp; // 更新当前采集到的最大值
        if (adc_val_tmp < get_adcmin)
            get_adcmin = adc_val_tmp; // 更新当前采集到的最小值
        adc_val_sum += adc_val_tmp;
    }

    adc_val_sum -= get_adcmax;        // 去掉一个最大
    adc_val_sum -= get_adcmin;        // 去掉一个最小
    adc_val_tmp = (adc_val_sum >> 4); // 除以16，取平均值

    return adc_val_tmp;
}

void sys_init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();
    timer1_config();
    timer2_pwm_config();

    adc_config();

    INTE |= (0x01 << 1); // 使能 timer1中断

    delay_ms(1); // 等待系统稳定
    GIE = 1;
}

// 红外按键处理
void ir_handle(void)
{
    if (flag_is_recved_data || flag_is_recv_ir_repeat_code)
    {
        static u8 last_ir_data = 0;

        /*
            如果有按键长按
            长按的按键是亮度加或亮度减，才执行，否则直接退出
        */
        if (flag_is_recv_ir_repeat_code)
        {
            // printf("recv repeat code 0x %bx\n", last_ir_data);
            // last_ir_data = ir_data;
            flag_is_recv_ir_repeat_code = 0;
            return;
        }

        // printf("recv ir data 0x %bx\n", ir_data);
        switch (ir_data)
        {
        case IR_KEY_ON:
            /* code */
            break;

        case IR_KEY_OFF:
            /* code */
            break;

        case IR_KEY_RED:
            /* code */
            break;

        case IR_KEY_FULL_BRIGHTNESS: // 全亮
            /* code */
            break;
        case IR_KEY_HALF_BRIGHTNESS: // 半亮
            /* code */
            break;
        case IR_KEY_BRIGHTNESS_ADD_OR_NUM_4: /* 亮度加，也是小遥控器的数字4 */
            /* code */
            break;

        case IR_KEY_BRIGHTNESS_SUB_OR_NUM_2: /* 亮度减，也是小遥控器的数字2 */
            /* code */
            break;

        case IR_KEY_AUTO_OR_NUM_5: /* 自动模式 ，也是小遥控器的数字5 */
            /* code */
            break;

        case IR_KEY_3H_OR_NUM_3: /* 3H，也是小遥控器的数字3 */
            /* code */
            break;
        case IR_KEY_5H_OR_M1: /* 5H，也是小遥控器的M1 */
            /* code */
            break;
        case IR_KEY_8H_OR_M3: /* 8H，也是小遥控器的M3 */
            /* code */
            break;

        case IR_KEY_SET: // SET 模式设置
            /* code */
            break;
        case IR_KEY_NUM_1: // 数字1
            /* code */
            break;

        case IR_KEY_M2: //
            /* code */
            break;

        default:
            break;
        }

        last_ir_data = ir_data;
        ir_data = 0;
        flag_is_recved_data = 0;
    }
}

// 电池放电控制，实际是控制 主灯光 对应的pwm占空比
void bat_discharge_handle(void)
{
    // 如果时间到来，减少pwm占空比
}

void main(void)
{
    sys_init();


    timer2_pwm_set_feq();

    while (1)
    {
    }
}

void int_isr(void) __interrupt
{
    __asm;
    movra _abuf;
    swapar _PFLAG;
    movra _statusbuf;
    __endasm;

    if (T1IF & T1IE) // 定时器1 中断，100us
    {

        // P22D = ~P22D;

#if 1     // 红外解码【需要放到100us的定时器中断来处理】
        { // 红外解码
            // static volatile u8 ir_fliter;
            static volatile u16 ir_level_cnt; // 红外信号的下降沿时间间隔计数
            static volatile u32 __ir_data;    //
            // static volatile u16 ir_long_press_cnt; // 檢測紅外遙控長按的計數值

            // 对每个下降沿进行计数
            if (ir_level_cnt <= 1300)
                ir_level_cnt++;

            // 滤波操作
            // ir_fliter <<= 1;
            // if (IR_RECV_PIN)
            // {
            //     ir_fliter |= 1;
            // }
            // ir_fliter &= 0x07;

            // if (ir_fliter == 0x07)
            //     filter_level = 1;
            // else if (ir_fliter == 0x00)
            //     filter_level = 0;

            // if (filter_level)
            if (IR_RECV_PIN)
            {
                last_level_in_ir_pin = 1; // 表示接收到的是高电平

                // 如果之前也是高电平
                if (ir_level_cnt > 1200) // 超时时间(120000us,120ms)
                {
                    // if (__ir_data != 0) // 超时，且接收到数据(红外接收处理函数中会把ir_data清零)
                    if (__ir_data != 0) // 超时，且接收到数据(现在是在中断服务函数中把__ir_data自动清零)
                    {
                        // // 带校验的版本：
                        // if ((u8)(__ir_data >> 8) == (u8)(~__ir_data)) // 如果键值的原码和反码相等
                        // {
                        // flag_is_recved_data = 1;
                        // }

                        // 不带校验的版本
                        if (0 == flag_is_recved_data)
                        {
                            // if ((__ir_data & 0xFF0000) == 0xFF0000)
                            {
                                ir_data = ~__ir_data;
                                __ir_data = 0;
                                flag_is_recved_data = 1;
                            }
                        }
                    }

                    flag_is_recv_ir_repeat_code = 0; // 认为遥控器按键已经按下，然后松开
                }
            }
            else
            {
                if (last_level_in_ir_pin)
                {
                    // 如果之前检测到的是高电平，现在检测到了低电平
                    if (ir_level_cnt <= 8) // 小于800us，说明接收到无效的数据，重新接收
                    {
                        // FLAG_IS_RECVED_ERR_IR_DATA = 1;
                        flag_is_recv_ir_repeat_code = 0;
                    }
                    else if (ir_level_cnt <= 18) // 小于1800us,说明接收到了逻辑0
                    {
                        __ir_data <<= 1;

                        // P15D = 0; // 测试红外解码
                        // P15D = ~P15D; // 测试红外解码
                    }
                    else if (ir_level_cnt <= 26) // 小于2600us,说明接收到了逻辑1
                    {
                        __ir_data <<= 1;
                        __ir_data |= 0x01;

                        // P15D = 1; // 测试红外解码
                    }
                    else if (ir_level_cnt <= 150) // 小于15000us,说明接收到了格式头
                    {
                        // FLAG_IS_RECVED_ERR_IR_DATA = 1;
                        // ir_long_press_cnt = 0; // 加上这一条会检测不到长按
                    }
                    else if (ir_level_cnt <= 420) // 小于42ms,说明接收完一次完整的红外信号
                    {
#if 0 // 带校验的版本，命令源码与命令反码进行校验
    
                if ((u8)(__ir_data >> 8) == (u8)(~__ir_data)) // 如果键值的原码和反码相等
                {
                    flag_is_recved_data = 1;
                    flag_is_recv_ir_repeat_code = 1; //
                    ir_long_press_cnt = 0;
                }

#else // 不带校验的版本

                        if (0 == flag_is_recved_data) // 如果之前未接收到数据/接收到的数据已经处理完毕
                        {
                            // if ((__ir_data & 0xFF0000) == 0xFF0000)
                            {
                                ir_data = ~__ir_data;
                                __ir_data = 0;
                                flag_is_recved_data = 1;
                                // flag_is_recv_ir_repeat_code = 1; //
                            }
                        }

#endif // 不带校验的版本
                    }
                    else if (ir_level_cnt <= 1200) // 小于120000,120ms,说明接收到了重复码
                    {
                        // if (ir_long_press_cnt < 65535)
                        //     ir_long_press_cnt++;

                        flag_is_recv_ir_repeat_code = 1;
                    }
                    // else // 超过120000,说明接收到无效的数据
                    // {
                    // }

                    ir_level_cnt = 0;
                }

                last_level_in_ir_pin = 0; // 表示接收到的是低电平
            }
        } // 红外解码
#endif // 红外解码【需要放到100us的定时器中断来处理】

#if 1                       // 控制LED指示灯
        {                   // 控制LED指示灯
            static u16 cnt; // 用软件实现PWM驱动LED的相关变量
            cnt++;

            if (cnt <= 20) // 20 * 100us
            {
                if (flag_is_led_1_enable)
                {
                    LED_1_PIN = LED_ON_LEVEL;
                }

                if (flag_is_led_2_enable)
                {
                    LED_2_PIN = LED_ON_LEVEL;
                }

                if (flag_is_led_3_enable)
                {
                    LED_3_PIN = LED_ON_LEVEL;
                }

                if (flag_is_led_4_enable)
                {
                    LED_4_PIN = LED_ON_LEVEL;
                }

                if (flag_is_led_5_enable)
                {
                    LED_5_PIN = LED_ON_LEVEL;
                }
            }
            else
            {
                LED_1_PIN = LED_OFF_LEVEL;
                LED_2_PIN = LED_OFF_LEVEL;
                LED_3_PIN = LED_OFF_LEVEL;
                LED_4_PIN = LED_OFF_LEVEL;
                LED_5_PIN = LED_OFF_LEVEL;
            }

            if (cnt >= 100) // 100 * 100us
            {
                cnt = 0;
            }

        } // 控制LED指示灯
#endif // 控制LED指示灯

        T1IF = 0;
    } //  if (T1IF & T1IE) // 定时器1 中断，100us

    //=======T3========================
    if (T3IF & T3IE)
    {
        // P14D = !P14D;
        T3IF = 0;
    }
    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
