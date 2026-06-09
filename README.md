# RPLidar Mecanum Car

基于 RPLidar 激光雷达与麦克纳姆轮底盘的小车控制项目。

本项目目前实现了麦克纳姆轮底盘基础运动标定、RPLidar 雷达数据读取、基础直行、简单横向纠偏和前方避障转向逻辑。当前版本为基础可运行版本，可作为后续路径规划、复杂赛道控制和 8 字轨迹优化的基础。

## 当前版本

**v0.11 - 基础可运行版本**

当前版本已完成：

* 底盘六种基础动作标定
* 雷达前、左、右方向数据读取
* 简单直行控制
* 简单左右纠偏
* 前方障碍检测与左右转向选择
* 雷达异常读取保护

## 硬件组成

* ESP32 控制板
* RPLidar 激光雷达
* 麦克纳姆轮小车底盘
* 四路电机驱动模块
* 自定义 `MecanumDriver` 驱动库

## 电机引脚定义

```cpp
#define MOTOR1_PIN1 9
#define MOTOR1_PIN2 8
#define MOTOR2_PIN1 12
#define MOTOR2_PIN2 13
#define MOTOR3_PIN1 11
#define MOTOR3_PIN2 10
#define MOTOR4_PIN1 46
#define MOTOR4_PIN2 21
```

## 电机通道标定

经过实测，`MecanumDriver` 的四路输出与实际轮子的对应关系如下：

| 输出通道  | 实际轮子 | 正值方向 |
| ----- | ---- | ---- |
| 第 1 路 | 左后轮  | 后转   |
| 第 2 路 | 右后轮  | 后转   |
| 第 3 路 | 右前轮  | 前转   |
| 第 4 路 | 左前轮  | 前转   |

因此程序在 `driveFixed()` 中对后轮方向进行了修正，使上层控制逻辑可以统一按照：

```text
fl / fr / bl / br 为正值时，对应轮子向前转
```

来理解。

目前已验证以下基础动作可以正常执行：

* 直行
* 后退
* 左转
* 右转
* 左平移
* 右平移

## 雷达角度定义

根据当前雷达安装方向，程序暂定：

```cpp
const int F_ANGLE = 180;
const int L_ANGLE = 110;
const int R_ANGLE = 250;
```

含义如下：

| 方向 | 雷达角度 |
| -- | ---- |
| 前方 | 180° |
| 左方 | 110° |
| 右方 | 250° |

程序使用：

```cpp
float lidarDistances[360] = {0};
```

保存 0° 到 359° 的雷达距离数据。每读取到一个雷达扫描点，就根据该点角度更新对应数组位置。

## 核心控制逻辑

程序当前包含三个主要状态：

```cpp
#define MODE_RUN         0
#define MODE_TURN_LEFT   1
#define MODE_TURN_RIGHT  2
```

### MODE_RUN

正常运行状态。小车会读取前方、左方、右方平均距离，并执行：

* 前方无障碍时直行
* 左侧距离过近时向右纠偏
* 右侧距离过近时向左纠偏
* 前方距离低于阈值时进入转向状态

### MODE_TURN_LEFT

左转状态。小车以固定转向速度左转，持续时间由 `TURN_DURATION` 控制。

### MODE_TURN_RIGHT

右转状态。小车以固定转向速度右转，持续时间由 `TURN_DURATION` 控制。

## 当前主要参数

```cpp
const float BASE_SPEED = 60.0;
const float CORRECT_SPEED = 20.0;
const float TURN_SPEED = 75.0;

const float FRONT_TURN_THRESH = 270;
const float SIDE_NEAR_THRESH = 170;
const float SIDE_DIFF_MARGIN = 100;

const float VALID_MIN = 0;

const int AVG_RANGE = 20;
const unsigned long TURN_DURATION = 900;
```

参数说明：

| 参数                  | 含义             |
| ------------------- | -------------- |
| `BASE_SPEED`        | 直行速度           |
| `CORRECT_SPEED`     | 横向纠偏速度         |
| `TURN_SPEED`        | 原地转向速度         |
| `FRONT_TURN_THRESH` | 前方触发转弯的距离阈值    |
| `SIDE_NEAR_THRESH`  | 侧方触发纠偏的距离阈值    |
| `SIDE_DIFF_MARGIN`  | 左右距离差触发纠偏的最小差值 |
| `AVG_RANGE`         | 雷达取平均时中心角左右范围  |
| `TURN_DURATION`     | 转弯持续时间         |

## 雷达异常保护

为了避免雷达偶发读取失败导致小车失控，程序加入了保护逻辑：

* 如果前方、左方、右方均无有效数据，小车停车保护。
* 雷达偶发读取失败时不立即重启。
* 当长时间无有效雷达点时，尝试重新启动雷达扫描。
* 控制逻辑不再依赖 `isNewScan`，而是每 50ms 执行一次。

## 当前状态

当前版本已经可以作为基础可运行版本使用：

* 底盘运动方向已标定完成。
* 雷达前、左、右方向数据可以正常读取。
* 小车可以完成基础直行、简单纠偏和前方避障转向。

## 已知问题

* 纠偏逻辑仍为简单阈值判断，暂未加入 PID 控制。
* 转弯目前依赖固定时间控制，尚未实现闭环角度控制。
* 转弯后暂未加入独立的直行锁定状态，复杂环境下可能仍会出现连续误判。
* 右前轮曾出现响应偏慢现象，后续可通过电机补偿或硬件检查继续优化。

## 后续计划

* 优化纠偏阈值和纠偏强度。
* 增加转弯后直行锁定状态。
* 根据实际赛道进一步调整雷达角度和距离阈值。
* 在基础运动稳定后，继续完善路径控制逻辑。
* 尝试加入 PID 或更平滑的比例纠偏策略。

## 版本记录

### v0.10

基础可运行版本。

* 完成麦克纳姆轮底盘运动标定。
* 完成 RPLidar 数据读取。
* 实现基础直行、简单纠偏和前方避障转向。
* 加入雷达异常保护和定时控制循环。
