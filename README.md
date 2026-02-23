# Vacuum Grasp Dashboard

真空抓取可靠性与自恢复模块 - 暗色工业控制台 Web Dashboard（3D可视化版本）

## 功能特性

- **14状态FSM**: 完整的抓取流程状态机
- **3D可视化**: three.js 实时渲染机械臂、VGC10吸盘阵列、托盘
- **离线可用前端依赖**: three.js / OrbitControls / Tailwind / Chart.js 全部本地托管
- **故障注入**: 支持预吸取失败、搬运掉压两种故障场景
- **自动重试**: 3x3网格偏移重试（中心→四邻→四角）
- **自动恢复**: 掉压检测后自动进入恢复模式
- **实时监控**: WebSocket 每30ms推送状态
- **暗色工业风格**: 专业控制台UI设计

## 项目结构

```
robert_arm_sensor/
├── backend/
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py          # FastAPI 入口
│   │   ├── fsm.py           # 状态机核心
│   │   ├── models.py        # 状态码/事件码枚举 + 数据模型
│   │   ├── simulator.py     # 模拟器 (VisionSim/RobotSim/GripSim)
│   │   └── ws_manager.py    # WebSocket管理
│   ├── requirements.txt
│   └── Dockerfile
├── frontend/
│   ├── index.html           # 单页应用 (three.js 3D工作区)
│   └── vendor/              # 本地前端依赖 (three/tailwind/chartjs)
├── docker-compose.yml
└── README.md
```

## 快速开始

### 前置要求

- Docker & Docker Compose
- Ubuntu 24.04 (推荐) 或其他 Linux 发行版

### 1. 克隆仓库

```bash
git clone https://github.com/cj-kai/robert_arm_sensor.git
cd robert_arm_sensor
```

### 2. 一键启动

```bash
docker compose up -d --build
```

### 3. 访问 Dashboard

打开浏览器访问: `http://<服务器IP>:8000`

## 3D 工作区操作

| 操作 | 鼠标 |
|------|------|
| 旋转视角 | 左键拖拽 |
| 缩放 | 滚轮 |
| 平移 | 右键拖拽 |

## 坐标系映射说明（关键）

**后端（工业约定）**:
- `x_m`: 左右方向
- `y_m`: 前后方向
- `z_m`: 高度方向

**前端（three.js）**:
- `three.x = x_m`（左右）
- `three.y = z_m`（高度）
- `three.z = y_m`（前后）

> 这种映射是因为 three.js 默认 Y 轴为高度，而工业机器人通常使用 Z 轴为高度。前端在接收 WS 数据时自动完成映射。

## Ubuntu 云端部署

### 1. 安装 Docker

```bash
# 更新包索引
sudo apt update

# 安装 Docker
curl -fsSL https://get.docker.com | sh

# 添加当前用户到 docker 组（可选，避免每次使用 sudo）
sudo usermod -aG docker $USER
newgrp docker
```

### 2. 开放端口

```bash
# 如果使用 ufw
sudo ufw allow 8000/tcp
sudo ufw reload

# 如果使用云服务商防火墙，请在控制台开放 8000 端口
```

### 3. 部署应用

```bash
# 克隆并启动
git clone https://github.com/cj-kai/robert_arm_sensor.git
cd robert_arm_sensor
docker compose up -d --build
```

### 4. 验证部署

```bash
# 检查容器状态
docker compose ps

# 查看日志
docker compose logs -f
```

## API 文档

### REST API

| 方法 | 路径 | 描述 |
|------|------|------|
| POST | `/api/fsm/start` | 启动 FSM |
| POST | `/api/fsm/stop` | 停止 FSM |
| POST | `/api/fsm/reset` | 重置 FSM |
| POST | `/api/fault/next_pre_suction_fail` | 注入故障：下一次预吸取失败 |
| POST | `/api/fault/drop_once` | 注入故障：搬运中掉压 |
| POST | `/api/log/clear` | 清空日志 |

### WebSocket

- 端点: `/ws`
- 推送频率: 30ms
- 推送格式:
```json
{
  "ts": 1708123456.789,
  "state": "PRE_SUCTION_CHECK",
  "vacuum_ok": true,
  "vacuum_kpa": -52.3,
  "retry_count": 0,
  "recover_count": 0,
  "success_count": 5,
  "last_event": "VACUUM_ON",
  "log": [...],
  "robot_pose": {"x_m": 0.25, "y_m": 0.10, "z_m": 0.15, "roll_deg": 0, "pitch_deg": 90, "yaw_deg": 0},
  "target_pose": {"x_m": 0.35, "y_m": 0.12, "z_m": 0.0},
  "place_pose": {"x_m": -0.25, "y_m": 0.25, "z_m": 0.05},
  "vision": {"detected": true, "confidence": 0.92},
  "grip": {"vacuum_on": true, "sealed": false},
  "fault": {"active": false, "code": "", "msg": ""}
}
```

## 状态码

| 状态 | 描述 |
|------|------|
| IDLE | 空闲 |
| DETECTING_TARGET | 检测目标 |
| PLANNING_APPROACH | 规划接近路径 |
| MOVING_TO_PREGRASP | 移动到预抓取位置 |
| DESCENDING_TO_CONTACT | 下降到接触 |
| PRE_SUCTION_CHECK | 预吸取检查 |
| LIFT_VERIFICATION | 抬起验证 |
| TRANSPORT_MONITORING | 搬运监控 |
| MOVING_TO_PLACE | 移动到放置位置 |
| RELEASING_LOAD | 释放负载 |
| RETURNING_HOME | 返回原点 |
| AUTO_RETRYING_GRASP | 自动重试抓取 |
| AUTO_RECOVERY_MODE | 自动恢复模式 |
| FAULT_LATCHED | 故障锁定 |

## 3D 场景元素

| 元素 | 描述 |
|------|------|
| CRX 机械臂 | 白色底座 + 3段连杆 + 绿色关节球 |
| VGC10 末端 | 灰色方盒 + 10个吸盘（状态变色） |
| 托盘 | 蓝色半透明（目标位置） |
| 放置区 | 绿色半透明（放置位置） |
| 标记球 | 红色（视觉检测点） |

### 吸盘状态颜色

| 状态 | 颜色 |
|------|------|
| 关闭 | 深灰 (#0f172a) |
| 真空开启 | 黄色 (#f59e0b) |
| 密封成功 | 绿色 (#22c55e) |

## 故障注入测试流程

### 测试预吸取失败 → 自动重试

1. 点击 **Next Pre-Suction Fail**
2. 点击 **Start**
3. 观察 FSM 进入 AUTO_RETRYING_GRASP，进行3x3网格偏移重试
4. 最多9次重试后进入 FAULT_LATCHED

### 测试搬运掉压 → 自动恢复

1. 点击 **Drop Once During Transport**
2. 点击 **Start**
3. 观察 FSM 在 TRANSPORT_MONITORING 阶段检测到掉压
4. 自动进入 AUTO_RECOVERY_MODE，回到 DETECTING_TARGET 重新开始

## 硬件替换

当硬件就绪时，只需替换以下文件：

- `backend/app/simulator.py` 中的 `VacuumSim` → 接入真实真空传感器
- `backend/app/simulator.py` 中的 `RobotSim` → 接入真实机器人控制器
- `backend/app/simulator.py` 中的 `VisionSim` → 接入 AruCo 或其他视觉系统
- `backend/app/simulator.py` 中的 `GripSim` → 接入真实夹具控制器

FSM 逻辑和 API 保持不变。

## 许可证

MIT License
