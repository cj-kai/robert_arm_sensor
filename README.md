# CRX Tray Washer Simulation Dashboard

用于演示 CRX 系列机械臂在托盘清洗工位中的流程仿真与调试界面（纯网页展示 + 后端仿真执行）。

当前阶段重点：

- 网页 Dashboard（静态 `index.html`）实时展示状态、日志、3D 场景
- Python 执行层（FastAPI）运行 FSM 和仿真器
- Node.js 网关（TypeScript）作为前端主入口，代理 API/WS，提升容错
- 纯仿真流程覆盖：`脏盘架 -> 洗机上料 -> 洗机等待 -> 回流取盘 -> 净盘架`
- 相机方案预留：腕部 USB 相机 + ArUco/AprilTag（当前为模拟状态）

## 架构概览

- `frontend/index.html`
  - 静态单页 Dashboard（不引入前端构建工具）
  - three.js 3D 工位演示
  - WebSocket 状态显示
- `gateway/`
  - Node.js + TypeScript 网关
  - 对外提供 `/ws` 和 `/api/*`
  - 代理 Python 执行层并在掉线时返回降级状态
- `backend/`
  - FastAPI 执行层
  - FSM、模拟机器人/视觉/真空/传送带适配器
  - 输出扩展状态（`joint_angles_rad`、`payload`、`washer` 等）

## 当前仿真默认参数（已内置）

- 相机工作距离：`0.40 m`（建议范围 `0.30 ~ 0.50 m`）
- 视觉方案：`Aruco`（预留真实接入）
- 洗机模型：`MIMASA DA-80`（仿真节拍默认 `12s`）
- 托盘/箱体仿真（3种）
  - `Square Tray 10.75"` (`0.273 x 0.273 x 0.0318 m`)
  - `Rect Tray 11.125" x 7.75"` (`0.283 x 0.197 x 0.0191 m`)
  - `Bin 16.625" x 11" x 5"` (`0.422 x 0.279 x 0.127 m`)

## 目录结构

```text
robert_arm_sensor/
├─ backend/
│  ├─ app/
│  │  ├─ main.py            # FastAPI 执行层（REST + WS）
│  │  ├─ fsm.py             # FSM 与流程状态转换
│  │  ├─ simulator.py       # Vision/Robot/Vacuum/Grip 模拟器
│  │  ├─ adapters.py        # 适配器接口与模拟实现
│  │  ├─ models.py          # 状态模型与 WS 载荷
│  │  ├─ config_store.py    # JSON 配置（默认参数/运行时更新）
│  │  └─ ws_manager.py      # WebSocket 管理
│  ├─ Dockerfile
│  └─ requirements.txt
├─ gateway/
│  ├─ src/index.ts          # Node 网关（API/WS 代理 + 静态页服务）
│  ├─ Dockerfile
│  ├─ package.json
│  └─ tsconfig.json
├─ frontend/
│  ├─ index.html            # Dashboard + three.js 场景
│  ├─ assets/               # 机器人/模型资源（可选）
│  └─ vendor/               # three.js / URDFLoader（可选）
├─ tools/
│  └─ robot_assets/         # 官方 FANUC 资源下载脚本（骨架）
├─ ops/
│  └─ edge/                 # 边缘机部署说明（占位）
├─ docker-compose.yml       # 默认使用 gateway + backend
└─ README.md
```

## 快速开始（推荐：Docker）

### 1. 启动

```bash
docker compose up -d --build
```

### 2. 打开页面

- 本机：`http://127.0.0.1:8000/`
- 云端：`http://<server-ip>:8000/`

### 3. 基本操作

- `Start`：启动一轮流程（脏盘 -> 洗机 -> 回流 -> 净盘）
- `Stop`：中止当前任务
- `Reset`：复位状态和计数器
- `Next Pre-Suction Fail`：注入“下次预吸失败”
- `Drop Once During Transport`：注入“运输中掉落”

## 本地开发运行（不使用 Docker）

### 方式 A：仅 Python 执行层（直连）

```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

说明：

- 这种方式会使用 `backend` 直接服务静态页（`backend/Dockerfile` 会复制 `frontend` 到 `backend/static`）
- 如果你本地直接运行 Python，建议优先使用下面的网关方式，保证和部署环境一致

### 方式 B：Node 网关 + Python 执行层（与部署一致）

终端 1（Python 执行层）：

```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

终端 2（Node 网关）：

```bash
cd gateway
npm install
$env:PYTHON_EXEC_BASE="http://127.0.0.1:8000"
npm run start
```

访问：

- `http://127.0.0.1:8000/`（由网关提供前端页面）

## WebSocket 与 API（对前端）

### WebSocket

- `GET /ws`
- 推送状态包含（已扩展）：
  - `joint_angles_rad`
  - `vision`
  - `grip`
  - `fault`
  - `conveyor`
  - `robot`
  - `task`
  - `payload`
  - `washer`

### 常用 API

- `POST /api/task/start_cycle`
- `POST /api/task/pause`
- `POST /api/task/resume`
- `POST /api/task/abort`
- `POST /api/fsm/reset`
- `POST /api/fault/next_pre_suction_fail`
- `POST /api/fault/drop_once`
- `POST /api/log/clear`
- `GET /api/system/health`
- `GET /api/config`
- `PUT /api/config`
- `GET /api/calibration/status`
- `GET /api/camera/stream.mjpg`（仿真模式为占位 MJPEG）

## 3D 场景说明

当前场景已按托盘清洗工位布局做简化示意：

- 左侧：脏盘架 / 净盘架
- 中间：机械臂工作区
- 右侧：洗机主体（DA-80 简化体块）+ 回流槽

可视化特性：

- 托盘尺寸根据 `payload.dims_m` 自动缩放
- 托盘颜色区分脏/净状态
- 洗机在处理阶段显示忙碌状态
- 3D 初始化失败不会拖垮按钮、日志、WS（容错降级）

## 故障注入与演示流程

### 预吸失败（自动重试）

1. 点击 `Next Pre-Suction Fail`
2. 点击 `Start`
3. 系统在 `PRE_SUCTION_CHECK` 失败后进入 `AUTO_RETRYING_GRASP`
4. 执行偏移重试（3x3 网格策略）

### 运输掉落（自动恢复）

1. 点击 `Drop Once During Transport`
2. 点击 `Start`
3. 系统在运输阶段检测掉落
4. 进入 `AUTO_RECOVERY_MODE` 并重新搜索/抓取（仿真）

## 云端部署（推荐：离线打包上传）

由于云端拉取 GitHub 可能不稳定，推荐本地打包上传。

### 1. 本地打包

Windows PowerShell（在仓库根目录）：

```powershell
tar -czf robert_arm_sensor_deploy.tar.gz `
  --exclude=.git `
  --exclude=.cursor `
  --exclude=gateway/node_modules `
  --exclude=gateway/dist `
  --exclude=backend/data `
  .
```

### 2. 上传到云端

示例（按你的用户名/路径替换）：

```bash
scp robert_arm_sensor_deploy.tar.gz user@<server-ip>:/tmp/
```

### 3. 云端解压并启动

```bash
mkdir -p ~/apps/robert_arm_sensor
tar -xzf /tmp/robert_arm_sensor_deploy.tar.gz -C ~/apps/robert_arm_sensor --strip-components=1
cd ~/apps/robert_arm_sensor
docker compose up -d --build
docker compose ps
```

### 4. 验证

```bash
curl http://127.0.0.1:8000/api/system/health
```

## GitHub 提交流程（建议）

```bash
git status
git add .
git commit -m "feat: add gateway + phase1 tray washer simulation flow"
git push origin main
```

如果云端无法 `git pull`，继续使用“本地打包上传”的部署方式。

## 后续真实接入（下一阶段）

本项目已预留以下方向，但当前版本仍以仿真为主：

- FANUC CRX-20iA/L + `R-30iB Mini Plus`
- FANUC ROS 2 Driver（边缘机 Ubuntu）
- 腕部 USB 相机 + ArUco/AprilTag
- 真空吸盘 IO（当前为模拟）
- 传送带启停 IO（当前为模拟）

## 注意事项

- 你截图如果看不到 `Task & Links` / `Camera Debug`，通常是旧部署或浏览器缓存，请先强制刷新（`Ctrl+F5`）
- 若 3D 区域空白但 UI 正常，查看浏览器 Console 是否有 three.js/CDN/WebGL 报错

