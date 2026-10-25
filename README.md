# ReflexSync

腱反射实验的同步肌电、惯性采集与闭环叩击控制系统。

ESP32-S3 固件配合 Python 上位机，包含 48 通道 EMG、角速度、锤头加速度采集，以及独立的编码器闭环电机控制。

## 系统组成

| 目标 | 功能 | 硬件 |
| --- | --- | --- |
| `master` | 48CH EMG / 2 kSPS、ESP-NOW 同步与转发、USB CDC | ESP32-S3、6 × ADS1298 |
| `slave-01` | 三轴角速度，约 1000 点/秒 | ESP32-S3 SuperMini、MPU6500 |
| `slave-02` | 三轴加速度原始计数，约 1000 点/秒 | ESP32-S3 SuperMini、ADXL375 |
| `motor` | 1 ms 编码器闭环控制，USB/BLE，速度突降后撤驱动 | ESP32-S3 N16R8、DRV8871、AB 编码器 |

两个惯性节点默认 CPU 80 MHz；INT 未接时由硬件定时器每 1 ms 唤醒 SPI 读取任务。ADXL375 为 Mode 3 / 2 MHz，内部 1600 Hz 更新、FIFO Bypass；每次读取最新值，不保留全部内部样本。待机与采集采用每 50 ms 接收 20 ms 的无线窗口，同步交换期间保持唤醒。

## 安装与运行

安装 [Pixi](https://pixi.sh/)，克隆后在仓库根目录运行：

```bash
git clone https://github.com/Haruka-Hu/ReflexSync.git
cd ReflexSync
pixi install --locked

# 48CH EMG / IMU 采集和 MVC 反馈
pixi run scope48-mvc --ads-gain 6

# 电机上位机：USB / BLE，手动连接
pixi run motor-host

# 电机分档测试计划：300–900°/s，各20次；仅预览
pixi run motor-sweep --dry-run
```

电机五档配置保存在 `hammer_profiles.json`，可在电机上位机中设置各档的回撤角度和目标速度。固件广播名为 `Rotary_Hammer_BLE`，上位机也识别缓存名 `Neuro_Hammer_BLE`。

## 构建与烧录

```bash
pixi run build-master
pixi run build-slave-01
pixi run build-slave-02
pixi run build-motor

# 列出端口；多块板连接时应明确指定目标端口
pixi run platformio device list
pixi run upload-slave-01 --upload-port <PORT>
pixi run upload-slave-02 --upload-port <PORT>
pixi run upload-motor --upload-port <PORT>

# Master 使用 TinyUSB，运行时与 ROM 引导串口可能不同
pixi run upload-master --port <RUNTIME_PORT> --rom-port <ROM_PORT>
```

将尖括号参数替换为实际串口名，例如 macOS `/dev/cu.usbmodem...`、Windows `COM3`。电机环境为 **16 MB Flash / 8 MB Octal PSRAM**；SuperMini Slave 为 4 MB Flash。Master 的 PSRAM、ADS1298 接线需与实际板卡对应。烧录前断开使用同一串口的上位机。

## 接线与电机操作

- 两个惯性节点 SPI：CS=GPIO10、MOSI=GPIO11、MISO=GPIO13、SCK=GPIO12。
- 电机：DRV8871 IN1/IN2=GPIO41/42，编码器 A/B=GPIO39/40，16 PPR × 四倍频 × 27:1。
- Master 引脚及 EMG 配置：`src/master/main.cpp`；电机实验参数：`src/motor/control_config.h`。
- 首次操作应确认物理零位并执行 ZERO，再从低速档验证行程。STOP 后需要重新确认并归零。
- 电机位置由 PCNT 硬件四倍频计数；V8 轨迹使用 CRC 校验、请求编号及缺点重读。
- 新电机协议为 `HAMMER_COAST_V9 / protocol=2`，使用 `SET / START / RETRACT / FIRE / STOP / TRACE` 等命令。

详细操作见 [电机控制说明](notes/电机控制说明.md)。编码器速度突降是接触候选，不等同于独立力测量；目标速度、允许输入范围也不代表机械验收结果。

## 文件结构

```text
src/master/             Master 入口与配置
src/slave-01/           角速度节点入口
src/slave-02/           加速度节点入口
src/motor/              电机控制、编码器、驱动和 USB/BLE 协议
lib/                    采集驱动、同步和无线协议
tools/scope48_mvc.py     采集 / MVC 上位机
tools/motor_controller.py 电机上位机入口
tools/hammer_*.py        电机界面、通信和协议
tools/analyze_data.py    录制后的 IMU 分析
tools/csv_mat_gui.py     CSV 转 MAT 与 EMG 通道相关性上位机
tools/csv_mat_core.py    CSV 读取、MAT 导出与相关性计算
tools/view_*.py          录制后的波形生成
scripts/                Master烧录辅助与电机分档测试
tests/                  电机轨迹链路、GUI 和编码器离线回归
hammer_profiles.json    电机五档配置
platformio.ini          固件板卡配置
pixi.toml / pixi.lock    Python / PlatformIO 环境和任务
```

## 文档

从 [文档索引](notes/文档索引.md) 开始，查看当前系统架构、接线与增益、构建烧录、采集界面、电机操作、协议和代码调用顺序。文档按当前源码重写，旧方案与历史实测已备份到仓库外。

## 数据与 GitHub 上传范围

默认录制保存到当前仓库的 `data/`，波形图片保存到 `outputs/waveforms/`。路径以脚本所在仓库为基准，与启动终端目录无关。界面显示完整保存路径；`--csv-dir` 可覆盖录制目录，相对路径仍以仓库根目录为基准。

仓库只跟踪源码、环境配置、五档参数和必要说明。`.gitignore` 排除 `data/`、`outputs/`、`diagnostics/`、构建产物、本地环境、CSV/BIN 等数据文件及本地历史记录。后续实验数据请保存在这些数据目录中。

检查结果与验证边界见 [发布检查说明](notes/发布检查说明.md)。

## CSV 转 MAT 与通道相关性

```bash
pixi run csv-mat
```

支持拖放或选择 EMG / IMU CSV、单文件及批量 MAT 转换；EMG 可选择通道、时间范围和滤波，查看相关性热力图并导出 MAT / CSV / PNG。使用现有 Pixi 环境运行 Python 图形脚本，详见 [使用说明](notes/CSV转MAT与通道相关性.md)。

## 电机离线回归

```bash
pixi run python -m unittest discover -s tests -v
```

包括轨迹分包故障注入、GUI 确认事务，以及真实 C++ 编码器／状态机的硬件模型测试。C++ 部分需系统提供 C++17 编译器；缺失时会跳过并明确报告。测试不连接或驱动设备，不代替实机重复叩击验收。

## 电机速度分档测试

`pixi run motor-sweep`通过BLE依次测试300、400、500、600、700、800、900°/s，每档20次；每击先低速HOME归零，默认回撤50°，逐击保存结果与CRC轨迹到本工程`outputs/motor_sweeps/`。Ctrl+C中止并尝试STOP。完整流程、USB用法和异常结果解释见[电机速度分档测试](notes/电机速度分档测试.md)。

### 批量叩击同步加速度

使用V11电机固件后，执行 `pixi run motor-sweep --master-port <Master串口>`，默认300—900°/s每档20次、逐击HOME、50°回撤；先用 `--speeds 300 --repeats 3` 联调。支持 `--analyze-run <输出目录>` 离线重算峰值重复性，详见 `notes/电机速度分档测试.md`。异常越界、超时、零点漂移或采集故障会终止整轮，不补敲。

### 已归并数据与肌电分析

2026-10-20的140次叩击原始记录已按七档整理到 [hammer_experiment_20261020](outputs/hammer_experiment_20261020/README.md)。MATLAB入口 `scripts/analyze_hammer_variability.m` 默认读取该目录。

肌电分解代码、参数配置和独立运行环境已迁到 [02-openhdemg](../../02-算法/02-openhdemg/README.md)。本工程 `scripts/` 保留硬件烧录、叩击采集和叩击分析入口。
