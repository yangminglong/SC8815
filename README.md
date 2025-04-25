## SC8815 通用固件库，可实现 SC8815 的全部功能。

    该库仿照 STM32 固件库的风格编写的，目前已通过初步测试，可以成功初始化 SC8815 并进行参数设置，但不保证所有函数的功能。

## Fork from [SC8815-Firmware-Library](https://github.com/Sghz-prages/SC8815-Firmware-Library)

#### 1. 库将提供以下实用函数：  
* 初始化 SC8815 的示例 Demo 函数  
* SC8815 硬件配置初始化函数  
* SC8815 读取中断状态函数  
* SC8815 读取内置ADC转换结果函数  
* SC8815 设置参数值函数  
* SC8815 获取参数设置值函数  
* SC8815 设置硬件配置函数  
* SC8815 获取硬件配置状态函数

#### 2.要使用此库，需要提供以下外部函数：  
```c
void I2C_WriteRegByte(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t ByteData); //通过I2C向设备寄存器写一个字节  
uint8_t I2C_ReadRegByte(uint8_t SlaveAddress, uint8_t RegAddress); //通过I2C从设备寄存器读一个字节  
void SoftwareDelay(uint8_t ms); //软件延时毫秒  
```

#### 3.在库的头文件中定义了 4 个硬件参数宏，您需要根据您的硬件参数设置：  
* SCHW_VBUS_RSHUNT：VBUS 电流路径上的感测电阻值，单位为mOhm（毫欧）  
* SCHW_BATT_RSHUNT：电池 电流路径上的感测电阻值，单位为mOhm（毫欧）

如果在您的应用中，OTG 时 VBUS 电压反馈的方式为 VBUS 引脚在芯片内部反馈，则只需要设置以上两个硬件参数宏。  

如果 OTG 时 VBUS 电压反馈的方式为 FB 引脚上外部分压电阻反馈，则需要设置以下两个硬件参数宏。  
* SCHW_FB_RUP：从 FB 连接到 VBUS 之间的电阻值，单位为Ohm（欧姆）  
* SCHW_FB_RDOWM：从 FB 连接到 GND 之间的电阻值，单位为Ohm（欧姆）  

库在设置 OTG 输出电压时会自动根据所使用的反馈方式计算 SC8815 寄存器参数，您只需要设置好硬件参数宏即可。

#### 5，完成几项简单的配置时候，便可以开始使用此库了。  

    建议从示例 Demo 函数中修改您需要的初始化参数，然后调用示例 Demo 函数进行初始化。

#### 6，注意事项：  
* 库是全中文注释，文件编码为GB2312，没有英文版本。  
* 硬件初始化结构体中的各项参数必须使用指定的宏定义值。  
* 库只对I2C进行操作，而 SC8815 的控制引脚（CE，PSTOP）需要您手动进行操作。  
* 库不会进行任何主动操作，SC8815 的中断处理是在中断发生后调用 SC8815_ReadInterrupStatus() 读取中断状态。
