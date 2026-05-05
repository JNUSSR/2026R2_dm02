# README

# JNUSSR R2 ON DM02

暨南大学 SSR 战队 ROBOCON 2026 赛季 R2 机器人 dm02 板 固件仓库。

本项目基于 robowalker_train 框架思路，融合 STM32CubeMX + CMake + FreeRTOS，面向 STM32H723 平台，作为队内通用板级支持与任务开发基础。
框架参考: https://github.com/yssickjgd/robowalker_train.git

> 使用 VS Code 时，如切换过工程路径或工具链，建议先清理旧的 CMake 缓存后再重新配置。
> 

## 特性

- MCU: STM32H723 系列
- RTOS: FreeRTOS (Middlewares/Third_Party/FreeRTOS)
- HAL: STM32H7xx HAL Driver
- 构建系统: CMake + Ninja (兼容 GCC Arm Embedded 工具链)
- 工程兼容: 同时保留 CubeMX 工程文件，便于继续用 CubeMX 维护外设
- 用户代码层: 按算法、驱动、模块、任务分层组织在 User_File 目录

## 目录说明

- Core: CubeMX 生成的核心启动与外设代码
- Drivers: CMSIS 与 HAL 驱动
- Middlewares: FreeRTOS、USB 等中间件
- USB_DEVICE: USB 设备层
- User_File: 队内业务层代码 (算法/驱动/设备/任务)
- cmake: 工具链与 CubeMX CMake 适配脚本
- build: 本地构建产物

## 构建说明 (CMake)

### 1. 安装依赖

- CMake >= 3.22
- Ninja
- gcc-arm-none-eabi 工具链

### 2. 配置与构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

默认目标名称为 `dm02_test`。

## dm02 迁移与兼容说明

### 1. FDCAN 与 A 板保持一致

dm02 的 FDCAN 配置策略与原 A 板保持一致即可，不需要重新定义一套新的总线协议。

- 继续使用经典帧 (Classic CAN) 与既有 ID 分配规则
- 继续沿用现有收发回调与打包逻辑 (CAN1/CAN2/CAN3)
- 迁移重点放在业务层参数，不放在底层总线协议重构

### 2. 输出量从“原始刻度”改为“真实物理扭矩”

新版本电机驱动中，`Target_Torque` 的语义为真实扭矩 (单位: Nm)，而不是历史上的电流/刻度近似量。

驱动内部通过电机模型把扭矩换算为电流，再映射为 CAN 下发刻度，典型计算过程如下:

```cpp
Out = (Target_Torque + Feedforward_Torque) / CURRENT_TO_TORQUE / Gearbox_Rate * CURRENT_TO_OUT;
```

说明:

- `CURRENT_TO_TORQUE`: 扭矩电流常数，表示电流与扭矩之间的物理关系
- `CURRENT_TO_OUT`: 电流到 CAN 输出刻度的映射系数
- `Gearbox_Rate`: 减速比 (直驱或无减速箱场景按对应模型处理)

在此移植版本中为适配A板R2代码的参数，Target_Torque直接以CAN 下发刻度赋值给Out

```bash
Out = Target_Torque;
```

### 3. 常用 DJI 电机参数 (当前工程实现)

1. C620 (对应 M3508)
- 最大输出刻度 `OUT_MAX`: 16384
- 扭矩电流常数 `CURRENT_TO_TORQUE`: `0.3 / (3591.0 / 187.0)`
- 电流到输出刻度系数 `CURRENT_TO_OUT`: `16384.0 / 20.0`
1. GM6020
- 最大输出刻度 `OUT_MAX`: 16384
- 扭矩电流常数 `CURRENT_TO_TORQUE`: `0.741`
- 电流到输出刻度系数 `CURRENT_TO_OUT`: `16384.0 / 3.0`
1. C610 (对应 M2006)
- 最大输出刻度 `OUT_MAX`: 10000
- 扭矩电流常数 `CURRENT_TO_TORQUE`: `0.18 / 36.0`
- 电流到输出刻度系数 `CURRENT_TO_OUT`: `10000.0 / 10.0`

### 4. GM6020 驱动版本说明

本仓库当前仅保留 GM6020 电流控制版本逻辑，并通过驱动版本枚举自动匹配发送 ID 规则。

迁移旧代码时请注意:

- 不再按旧版“电压控制输出”方式理解目标量
- 统一按“目标扭矩 (Nm) -> 电流 -> CAN 刻度”链路进行控制

### 5. CAN的用户回调变化

迁移旧代码时请注意:

- 用户的CAN的回调函数传入参数由`Struct_CAN_Rx_Buffer *Rx_Buffer`变为了`FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer`

### 6. DMA的使用

- 因为普通DMA无法通过外设总线访问DTCMRAM，所以须经过DMA搬运的数据必须用**`attribute**((section(".ram_d2")))`修饰，将数据搬运到ram的D2区域

Eg:

```cpp
__attribute__((section(".ram_d2"))) Struct_UART_Manage_Object UART1_Manage_Object = {nullptr};
__attribute__((section(".ram_d2"))) Struct_UART_Manage_Object UART2_Manage_Object = {nullptr};
```

```cpp
__attribute__((section(".ram_d2"))) Struct_SPI_Manage_Object SPI1_Manage_Object = {nullptr};
__attribute__((section(".ram_d2"))) Struct_SPI_Manage_Object SPI2_Manage_Object = {nullptr};
```

## 开发建议

- 新业务代码优先放在 User_File 目录，并按现有分层维护
- 尽量避免直接改动 CubeMX 自动生成区块，优先在 USER CODE 段扩展
- 调 PID 或功率控制时，优先校准扭矩模型参数，再调环路参数
- 出现异常优先检查: CAN ID 映射、减速比参数、扭矩常数与单位一致性

## 许可证

如无额外声明，按仓库后续约定执行。