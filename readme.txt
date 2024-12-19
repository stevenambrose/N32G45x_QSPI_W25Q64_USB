1. 功能说明
    USB 使用外部W25Q64模拟u盘，USB+QSPI+W25Q64
	使用W25Q64后4M储存数据,地址从0x40_0000至0x7f_fff0，如果需要全8M使用可以打开<mass_mal.c>将FLASH_OFFSET改成0x0,Mass_Block_Count[0]改成2048

2. 使用环境
    硬件环境：KEIL v5.36
    开发板：    N32G4XM_STB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. USBClock: 48MHz
			QSPI_CS----------------------PC10
			QSPI_CLK---------------------PC11
			QSPI_IO0---------------------PC12
			QSPI_IO1---------------------PD00
			QSPI_IO2---------------------PD01
			QSPI_IO3---------------------PD02

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行；
         2. 通过 USB 线连接 J3 USB 口，USB 挂载完成后，识别成 U 盘设备

4. 注意事项
    首次挂载 U 盘需要格式化，格式化完成后即可当成 U 盘使用

1. Function description
    USB uses EXTERNAL FLASH W25Q64 to simulate U disk，USB+QSPI+W25Q64
	Using W25Q64 last 4MB to save data,Address from 0x40_0000 to 0x7f_fff0，if want to use full 8MByte，open <mass_mal.c>,change FLASH_OFFSET to 0x0 and Mass_Block_Count[0] to 2048.

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32G4XM_STB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. USBClock: 48MHz
			QSPI_CS----------------------PC10
			QSPI_CLK---------------------PC11
			QSPI_IO0---------------------PC12
			QSPI_IO1---------------------PD00
			QSPI_IO2---------------------PD01
			QSPI_IO3---------------------PD02
                  
    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. Connect the J3 USB port through a USB cable, and after the USB is mounted, it will be recognized as a U disk device.
 
4. Matters needing attention
    The first mount U disk needs to be formatted, and it can be used as a U disk after formatting.