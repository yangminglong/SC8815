#include <Arduino.h>
#include "SC8815.h"
#include <Wire.h> //包含I2C库

#define SC8815_INT_PIN 2 //中断引脚定义, 这里使用数字引脚2作为中断引脚

//通过I2C向设备寄存器写一个字节
void I2C_WriteRegByte(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t ByteData)   
{
    Wire.beginTransmission(SlaveAddress); //开始I2C传输
    Wire.write(RegAddress);                //写入寄存器地址
    Wire.write(ByteData);                  //写入数据
    Wire.endTransmission();                //结束I2C传输
}

//通过I2C从设备寄存器读一个字节
uint8_t I2C_ReadRegByte(uint8_t SlaveAddress, uint8_t RegAddress)                   
{
    Wire.beginTransmission(SlaveAddress); //开始I2C传输
    Wire.write(RegAddress);                //写入寄存器地址
    Wire.endTransmission();                //结束I2C传输

    Wire.requestFrom(SlaveAddress, 1);     //请求从设备读取一个字节
    if (Wire.available())                  //如果有数据可读
    {
        return Wire.read();                //返回读取的数据
    }
    return 0;                              //如果没有数据可读，返回0
}

void SoftwareDelay(uint8_t ms)                                                      //软件延时毫秒
{
    delay(ms);                            //使用Arduino的delay函数进行延时
}


bool SC8815_InterruptFlag = false; //中断标志位
// 中断处理函数
void SC8815_InterruptHandler() {
  SC8815_InterruptFlag = true; //设置中断标志位为真
}

/****************************************
* @brief    初始化 SC8815 的示例 Demo 函数
*****************************************/
void SC8815_Init_Demo(void)
{
    SC8815_BatteryConfigTypeDef SC8815_BatteryConfigStruct = { 0 };
    SC8815_HardwareInitTypeDef SC8815_HardwareInitStruct = { 0 };
    SC8815_InterruptStatusTypeDef SC8815_InterruptMaskInitStruct = { 0 };

    /****启动 SC8815...****/
    //->设置 PSTOP 为高
    //->设置 CE 为低
    //SoftwareDelay(5);   //必要的启动延时

    //配置 SC8815 电池参数选项
    SC8815_BatteryConfigStruct.IRCOMP = SCBAT_IRCOMP_20mR;
    SC8815_BatteryConfigStruct.VBAT_SEL = SCBAT_VBAT_SEL_Internal;
    SC8815_BatteryConfigStruct.CSEL = SCBAT_CSEL_4S;
    SC8815_BatteryConfigStruct.VCELL = SCBAT_VCELL_4v25;
    SC8815_BatteryConfig(&SC8815_BatteryConfigStruct);

    //配置 SC8815 硬件参数选项
    SC8815_HardwareInitStruct.IBAT_RATIO = SCHWI_IBAT_RATIO_6x;
    SC8815_HardwareInitStruct.IBUS_RATIO = SCHWI_IBUS_RATIO_3x;
    SC8815_HardwareInitStruct.VBAT_RATIO = SCHWI_VBAT_RATIO_12_5x;
    SC8815_HardwareInitStruct.VBUS_RATIO = SCHWI_VBUS_RATIO_12_5x;
    SC8815_HardwareInitStruct.VINREG_Ratio = SCHWI_VINREG_RATIO_100x;
    SC8815_HardwareInitStruct.SW_FREQ = SCHWI_FREQ_300KHz_2;
    SC8815_HardwareInitStruct.DeadTime = SCHWI_DT_40ns;
    SC8815_HardwareInitStruct.ICHAR = SCHWI_ICHAR_IBAT;
    SC8815_HardwareInitStruct.TRICKLE = SCHWI_TRICKLE_Enable;
    SC8815_HardwareInitStruct.TERM = SCHWI_TERM_Enable;
    SC8815_HardwareInitStruct.FB_Mode = SCHWI_FB_Internal;
    SC8815_HardwareInitStruct.TRICKLE_SET = SCHWI_TRICKLE_SET_70;
    SC8815_HardwareInitStruct.OVP = SCHWI_OVP_Enable;
    SC8815_HardwareInitStruct.DITHER = SCHWI_DITHER_Disable;
    SC8815_HardwareInitStruct.SLEW_SET = SCHWI_SLEW_1mV_us;
    SC8815_HardwareInitStruct.ADC_SCAN = SCHWI_ADC_Disable;
    SC8815_HardwareInitStruct.ILIM_BW = SCHWI_ILIM_BW_1_25KHz;
    SC8815_HardwareInitStruct.LOOP = SCHWI_LOOP_Normal;
    SC8815_HardwareInitStruct.ShortFoldBack = SCHWI_SFB_Enable;
    SC8815_HardwareInitStruct.EOC = SCHWI_EOC_1_25;
    SC8815_HardwareInitStruct.PFM = SCHWI_PFM_Disable;
    SC8815_HardwareInit(&SC8815_HardwareInitStruct);

    //配置 SC8815 中断屏蔽选项
    SC8815_InterruptMaskInitStruct.AC_OK = sENABLE;
    SC8815_InterruptMaskInitStruct.INDET = sENABLE;
    SC8815_InterruptMaskInitStruct.VBUS_SHORT = sENABLE;
    SC8815_InterruptMaskInitStruct.OTP = sENABLE;
    SC8815_InterruptMaskInitStruct.EOC = sENABLE;
    SC8815_ConfigInterruptMask(&SC8815_InterruptMaskInitStruct);
    /***现在可以设置 PSTOP 引脚为低, 启动 SC8815 功率转换****/



    /*** 示例1, 设置为充电模式,电池和 VBUS 限流 2A, VINREG 设置为 12V ****/
    SC8815_SetBatteryCurrLimit(2000);
    SC8815_SetBusCurrentLimit(2000);
    SC8815_VINREG_SetVoltage(12000);
    SC8815_OTG_Disable();


    /*** 示例2, 设置为反向放电模式,电池和 VBUS 限流 3A, 输出电压 设置为 12V ****/
    //SC8815_SetBatteryCurrLimit(2000);
    //SC8815_SetBusCurrentLimit(2000);
    //SC8815_SetOutputVoltage(12000);
    //SC8815_OTG_Enable();

    // 初始化中断引脚
    pinMode(SC8815_INT_PIN, INPUT_PULLUP); //设置中断引脚为输入模式,上拉电阻
    attachInterrupt(digitalPinToInterrupt(SC8815_INT_PIN), SC8815_InterruptHandler, FALLING); //设置中断触发方式为下降沿触发

    Serial.println("SC8815 Initialized!");
}

void processInterrupt() {
  // 处理中断事件
  if (SC8815_InterruptFlag) {
    SC8815_InterruptStatusTypeDef InterruptStatusStruct = { 0 };
    SC8815_ReadInterrupStatus(&InterruptStatusStruct); //读取中断状态

    Serial.println("SC8815 Interrupt Occurred!");
    Serial.printf("AC_OK: %d, INDET: %d, VBUS_SHORT: %d, OTP: %d, EOC: %d\n",
                  InterruptStatusStruct.AC_OK,
                  InterruptStatusStruct.INDET,
                  InterruptStatusStruct.VBUS_SHORT,
                  InterruptStatusStruct.OTP,
                  InterruptStatusStruct.EOC);

    SC8815_InterruptFlag = false; //重置中断标志位
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Hello, world!");

  SC8815_Init_Demo();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);

  processInterrupt(); //处理中断事件

  /*** 读取 SC8815 ADC 数据 ****/
  uint16_t VbusVolt = SC8815_Read_VBUS_Voltage();
  uint16_t VbusCurr = SC8815_Read_VBUS_Current();
  uint16_t BattVolt = SC8815_Read_BATT_Voltage();
  uint16_t BattCurr = SC8815_Read_BATT_Current();

  Serial.printf("VBUS Voltage: %d mV, VBUS Current: %d mA, Battery Voltage: %d mV, Battery Current: %d mA\n", VbusVolt, VbusCurr, BattVolt, BattCurr);
}


