# 使用 KEIL5 + VSCode 开发

使用教程参考[立创·地正星](https://lceda001.feishu.cn/wiki/PxQCwDEWqiEC3gkU0kbcuEpenDe)

## 文件包含
- **basic-car**
  - BSP （各种驱动）
    -  eMPL（mpu6050的DMP库）
    -  mpu6050
    -  adc
    -  Filter算法
    -  gw（感为8路循迹）
    -  motor（小车控制）
    -  pid（simple-pid）
    -  i2c(软件i2c)
  - board.c （两个板级驱动文件，初始化，printf与串口重定向）
  - board.h
  - empty.c （main.c）
  - empty.syscfg （配置）
- **ti-24-car**
  - BSP （各种驱动）
    -  jy901b
    -  adc
    -  Filter算法
    -  gw（感为8路循迹）
    -  motor（小车控制）
    -  pid（simple-pid）
    -  i2c(硬件i2c)
  - board.c （两个板级驱动文件，初始化，printf与串口重定向）
  - board.h
  - empty.c （main.c）
  - empty.syscfg （配置）
