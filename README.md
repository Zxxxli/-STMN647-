# STM32N6 AI 多姿态估计运动检测系统

基于 **STM32N647** 开发板的实时 AI 运动姿态检测系统，支持深蹲、开合跳、平板支撑三种运动的自动识别、计数与质量评估，并通过 4G/5G 模组将数据上报至华为云 IoTDA，配合 Web 仪表板实现远程可视化。

---

## 目录结构

```
├── Appli/                          # STM32 应用程序（核心代码）
│   ├── Core/
│   │   ├── Inc/                    # 头文件
│   │   │   ├── app.h               # 应用主入口
│   │   │   ├── app_config.h        # 应用配置（运动参数、阈值等）
│   │   │   ├── app_bqueue.h        # 缓冲队列
│   │   │   ├── app_camera.h        # 摄像头接口
│   │   │   ├── app_cpuload.h       # CPU 负载监控
│   │   │   ├── app_lcd.h           # LCD 显示
│   │   │   ├── app_postprocess.h   # AI 后处理
│   │   │   ├── app_utils.h         # 工具函数
│   │   │   ├── fibocom.h           # Fibocom 4G/5G 模组
│   │   │   ├── main.h              # HAL 配置
│   │   │   ├── postprocess_conf.h  # YOLOv2 后处理参数
│   │   │   ├── stm32n6xx_hal_conf.h
│   │   │   └── stm32n6xx_it.h
│   │   └── Src/                    # 源文件
│   │       ├── app.c               # 主应用逻辑 & 运动检测算法（三大运动）
│   │       ├── app_bqueue.c        # 缓冲队列实现
│   │       ├── app_camera.c        # 摄像头驱动
│   │       ├── app_cpuload.c       # CPU 负载测量
│   │       ├── app_lcd.c           # LCD 显示驱动
│   │       ├── app_postprocess.c   # AI 推理后处理
│   │       ├── fibocom.c           # Fibocom AT 指令通信
│   │       ├── main.c              # 程序入口
│   │       ├── secure_nsc.c        # 安全非安全调用
│   │       └── stm32n6xx_it.c      # 中断服务
│   ├── STM32_MW_CAMERA/            # 摄像头中间件配置
│   ├── STM32_MW_ISP/               # ISP 图像处理配置
│   └── ThreadX/                    # ThreadX RTOS 配置
│
├── iot-screen/                     # Web 可视化看板
│   ├── index.html                  # 前端仪表板（科技感 UI）
│   └── backend/                    # Node.js 后端
│       ├── app.js                  # Express 服务入口
│       ├── config.js               # 配置（从 .env 加载）
│       ├── package.json
│       ├── cache/dashboardCache.js # 内存缓存层
│       ├── routes/dashboard.js     # REST API 路由
│       └── services/
│           ├── iotda.js            # 华为云 IoTDA 客户端
│           └── signer.js           # AK/SK 签名工具
│
├── .gitignore
├── LICENSE                         # MIT License
└── README.md                       # 本文件
```

---

## 功能特性

- **三种运动自动识别与计数**
  - 深蹲 (Squat)：基于膝盖角度判断下蹲/站立状态，支持深度评估和背部倾斜度检测
  - 开合跳 (Jumping Jack)：基于肩宽比、手臂位置、腿部分开角度综合判断
  - 平板支撑 (Plank)：检测身体水平角度、手肘-手腕位置，实时计时

- **AI 运动质量评估**
  - 每次动作完成后自动评分（深度、舒展度、姿态标准度）
  - 语音反馈激励（目标达成提醒、质量提示）

- **数据上云与远程可视化**
  - Fibocom 4G/5G 模组上报至华为云 IoTDA
  - Node.js 中继服务轮询设备影子
  - Web 实时仪表板展示运动数据

- **基于 ThreadX 实时操作系统**
  - 多线程流水线：NN 推理 → 后处理 → 显示 → ISP
  - NPU（神经网络处理单元）硬件加速

---

## 硬件需求

| 组件 | 型号 | 说明 |
|------|------|------|
| 开发板 | **正点原子 N647 开发板** (ATK-CNN647B) | 搭载 STM32N647 MCU |
| 摄像头 | **IMX335** 图像传感器 | 5MP，用于视频采集 |
| 显示 | **800×480 LCD** 屏幕 | 实时显示运动画面与数据 |
| 通信模组 | **Fibocom 4G/5G** 模块 | 通过 UART1 连接，可选 |
| 外部存储 | **MX25UM25645G** NOR Flash + HyperRAM | 存放模型与运行数据 |

---

## 软件环境与依赖

**本仓库仅包含核心应用代码（Appli）和 Web 可视化代码（iot-screen），编译运行需配合正点原子 SDK 和 STM32Cube 生态。**

### 必需依赖（需自行获取）

| 依赖 | 来源 | 说明 |
|------|------|------|
| **正点原子 N647 板级支持包** | 正点原子官方资料包 | 包含 Drivers/BSP、Middlewares、FSBL、External_Loader 等 |
| **STM32Cube_FW_N6_V1.0.0** | ST 官网或正点原子资料包 | STM32N6 HAL 驱动、CMSIS |
| **AI 模型文件** | 正点原子 SDK | YOLOv8 姿态估计模型（.h5 / npu 格式） |
| **STM32CubeIDE** | ST 官网 | 推荐版本 ≥ 1.16.0，用于编译和烧录 |

### Node.js 运行环境

| 依赖 | 版本要求 | 说明 |
|------|---------|------|
| Node.js | ≥ 18.x | 运行 iot-screen 后端 |
| npm | ≥ 9.x | 包管理 |

---

## 快速上手

### 1. 搭建 STM32 固件工程

```bash
# 步骤一：获取正点原子 SDK 完整包（请联系正点原子或访问 www.openedv.com 下载）
# 将本仓库 Appli/ 目录复制到 SDK 完整工程的对应位置：
#   <SDK_Root>/Projects/99_Applications/994_AI_Multi_Pose_Estimation/Appli/

# 步骤二：确保以下依赖目录存在（来自 SDK）
#
#   Drivers/              - HAL 及板级驱动
#   Middlewares/          - AI/ThreadX/FATFS/NetX 等中间件
#   FSBL/                 - 一级启动加载器
#   STM32Cube_FW_N6_V1.0.0/ - STM32N6 HAL 库

# 步骤三：用 STM32CubeIDE 导入工程
#   打开 STM32CubeIDE → File → Import → Existing Projects
#   选择 SDK 完整目录下的
#   Projects/99_Applications/994_AI_Multi_Pose_Estimation/STM32CubeIDE/
```

### 2. 编译 & 烧录

烧录顺序（STM32N6 双阶段启动）：

1. **先烧录 FSBL**（First Stage Boot Loader），初始化外部存储器
2. **再烧录 Appli**（应用程序），AI 运动检测固件

具体操作请参考正点原子 N647 开发板用户手册。

### 3. 启动 Web 可视化看板

```bash
# 进入 iot-screen 目录
cd iot-screen/backend

# 安装依赖
npm install

# 配置华为云认证（从 .env.example 复制）
cp .env.example .env
# 编辑 .env，填入华为云 IAM 认证信息和设备 ID

# 启动服务
node app.js

# 浏览器访问
# http://localhost:3001
```

### 4. 环境变量说明

编辑 `iot-screen/backend/.env`：

```env
# 华为云 IAM 认证
HW_IAM_NAME=your_iam_username
HW_IAM_PASSWORD=your_iam_password
HW_IAM_DOMAIN=your_iam_domain

# 项目与设备
HW_PROJECT_ID=your_project_id
HW_REGION=cn-north-4
DEVICE_ID=your_device_id

# IoTDA 接入
HW_IOTDA_INSTANCE_ID=your_instance_id
HW_IOTDA_APP_ENDPOINT=your_iotda_app_endpoint

# 服务端口
PORT=3001
```

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    STM32N647 (端侧推理)                      │
│                                                             │
│  摄像头(IMX335)                                             │
│     ↓                                                       │
│  ISP 图像处理                                               │
│     ↓                                                       │
│  NPU 推理 (YOLOv8 姿态估计)   ← ← ← AI 模型 (来自 SDK)      │
│     ↓                                                       │
│  后处理 (关键点提取)                                         │
│     ↓                                                       │
│  运动检测引擎                                                │
│  ├─ 深蹲识别 & 质量评估                                      │
│  ├─ 开合跳识别 & 质量评估                                    │
│  └─ 平板支撑识别 & 质量评估                                  │
│     ↓                                                       │
│  LCD 显示 (实时反馈)                                         │
│  Fibocom 4G/5G (数据上报)  ──→  华为云 IoTDA                 │
└─────────────────────────────────────────────────────────────┘
                                        ↓
                               ┌──────────────────┐
                               │ Node.js 中继服务  │
                               │  轮询设备影子      │
                               │  缓存最新数据      │
                               └──────────────────┘
                                        ↓
                               ┌──────────────────┐
                               │ Web 仪表板        │
                               │ 实时运动数据可视化 │
                               └──────────────────┘
```

---

## 运动检测算法说明

### 关键点定义（17个关键点，COCO 格式）

- 头部: 0-4 (鼻、眼、耳)
- 手臂: 5-10 (肩、肘、腕)
- 躯干: 11-12 (髋)
- 腿部: 13-16 (膝、踝)

### 深蹲检测

通过膝盖角度（髋-膝-踝三点）判断站/蹲状态切换：
- 膝盖角 < 110° → 下蹲
- 膝盖角 > 145° → 站立
- 同时评估：下蹲深度百分比、背部倾斜度

### 开合跳检测

综合四项指标：
- 肩宽比（肩宽/髋宽）
- 手腕是否高于肘部
- 肘部是否高于肩部
- 腿部打开角度

### 平板支撑检测

- 身体水平角度
- 手肘-手腕-肩膀位置关系
- 髋部下坠检测

---

## 技术栈

| 领域 | 技术 |
|------|------|
| MCU | STM32N647 (Arm Cortex-M55 + NPU) |
| 实时系统 | Azure ThreadX |
| AI 框架 | STM32 AI (NanoEdge AI / STM32Cube.AI) |
| 视觉模型 | YOLOv8 姿态估计 (Multi-Pose Estimation) |
| 图像处理 | STM32 ISP (Image Signal Processor) |
| 云平台 | 华为云 IoTDA (设备接入与影子管理) |
| 后端 | Node.js + Express |
| 前端 | HTML/CSS/JavaScript (原生) |
| 通信协议 | Fibocom AT 指令 / MQTT → IoTDA |

---

## 许可证

本项目采用 **MIT License**，详情参见 [LICENSE](LICENSE) 文件。

### 第三方组件许可

本项目使用了以下第三方软件/库，请遵守其各自的许可证条款：

- **STM32 HAL 驱动 & CMSIS** — STMicroelectronics（ST 专有许可）
- **Azure ThreadX** — Microsoft（MIT License）
- **FatFs** — ChaN (FatFs 对开源项目免费)
- **ST 中间件** (AI/ISP/Camera) — STMicroelectronics
- **Node.js 包** (Express, Axios, CORS 等) — 各自 MIT/ISC 许可

---

## 相关资源

- [正点原子官网](https://www.alientek.com)
- [正点原子技术论坛](https://www.openedv.com)
- [STM32N6 系列官方页面](https://www.st.com/stm32n6)
- [华为云 IoTDA 文档](https://support.huaweicloud.com/iotda)

---

## 免责声明

本仓库中的代码仅供学习和参考。第三方软件和中间件的版权归其各自所有者所有。使用前请确保已获得正点原子官方开发板支持包和 ST 官方软件包的合法授权。
