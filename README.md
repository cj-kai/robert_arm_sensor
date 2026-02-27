# CRX Tray Washer Simulation Dashboard

用于演示 CRX 机械臂托盘流程的网页仿真平台（前端可视化 + Node 网关 + Python 执行层）。

## 1. 项目目标

本项目聚焦于 Phase-1 流程闭环验证：

1. 识别脏托盘  
2. 侧吸抓取  
3. 放到清洗线  
4. 清洗回流  
5. 再抓取并放回净盘架  
6. 支持故障注入与恢复

当前以仿真为主，架构已预留真实设备接入路径（ROS2/FANUC、USB 相机、IO）。

## 2. 架构分层

```text
frontend/index.html (Three.js + Dashboard)
        │  WS /ws + REST /api/*
        ▼
gateway/src/index.ts (Node.js/TypeScript)
  - API代理 /api/* -> Python /internal/*
  - WS桥接 /ws <-> /internal/ws
  - 命令串行化 + Python断线降级
        ▼
backend/app/main.py (FastAPI 执行层)
  - SidePickExecutionFSM
  - VisionGuidedSidePickFSM
  - IK Solver (ikpy)
  - Sim Adapters / WS 广播
```

## 3. 目录说明

```text
backend/
  app/main.py                 FastAPI入口（REST+WS+MJPEG）
  app/side_pick_runtime.py    运行时封装：状态映射/任务控制
  app/vision_side_pick_fsm.py 核心状态机与轨迹规划
  app/ik_solver.py            IK求解（URDF+ikpy）
  app/models.py               WS状态模型(SystemState)
  app/adapters.py             Robot/Vision/Vacuum/Conveyor 适配层
  app/simulator.py            仿真组件
gateway/
  src/index.ts                网关核心（代理、桥接、降级）
frontend/
  index.html                  页面+3D+通信主逻辑（无构建工具）
  assets/robot/crx20ial/      URDF 与 CRX mesh 资源
tools/robot_assets/
  fetch_fanuc_crx20_assets.ps1 FANUC 资源下载脚本
ops/edge/README.md            边缘机部署说明
```

## 4. 快速启动

### 4.1 Docker（推荐）

```bash
docker compose up -d --build
```

访问：

- 本机：`http://127.0.0.1:8000/`
- 云端：`http://<server-ip>:8000/`

### 4.2 本地开发（无 Docker）

终端1（Python 执行层）：

```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

终端2（Node 网关）：

```bash
cd gateway
npm install
# Windows PowerShell:
$env:PYTHON_EXEC_BASE="http://127.0.0.1:8000"
npm run start
```

访问：`http://127.0.0.1:8000/`

## 5. 常用接口

### 5.1 WebSocket

- `GET /ws`

状态载荷包含：`state`、`joint_angles_rad`、`vision`、`grip`、`fault`、`task`、`payload`、`tray_body`、`washer` 等。

### 5.2 REST

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
- `GET /api/camera/stream.mjpg`

## 6. URDF/资产说明

- 主 URDF：`frontend/assets/robot/crx20ial/robot.urdf`
- mesh 目录：`frontend/assets/robot/crx20ial/meshes/crx20ial/...`
- 资产下载脚本：`tools/robot_assets/fetch_fanuc_crx20_assets.ps1`

PowerShell 示例：

```powershell
powershell -ExecutionPolicy Bypass -File tools/robot_assets/fetch_fanuc_crx20_assets.ps1 -OutDir "frontend/assets/robot/crx20ial"
```

## 7. 固定发布流程（本地打包 -> 云端部署）

### 7.1 本地打包（Windows PowerShell）

```powershell
cd D:\robert_arm_sensor
tar -czf robert_arm_sensor_deploy.tar.gz backend frontend gateway ops tools docker-compose.yml README.md .dockerignore .gitignore
```

上传：

```powershell
scp D:\robert_arm_sensor\robert_arm_sensor_deploy.tar.gz ubuntu@117.72.52.43:~/
```

### 7.2 云端重部署

```bash
rm -rf ~/robert_arm_sensor_new
mkdir -p ~/robert_arm_sensor_new
tar -xzf ~/robert_arm_sensor_deploy.tar.gz -C ~/robert_arm_sensor_new

cd ~/robert_arm_sensor_new
docker-compose down --remove-orphans
docker-compose build --no-cache backend gateway
docker-compose up -d
docker-compose ps
```

### 7.3 云端验版本（关键代码命中）

```bash
grep -n "orientation_mode\"] = \"all\"" ~/robert_arm_sensor_new/backend/app/ik_solver.py
grep -n "ee.add(tray)" ~/robert_arm_sensor_new/frontend/index.html
grep -n "trayAttached = backendBoundToTool" ~/robert_arm_sensor_new/frontend/index.html
```

## 8. 文档索引

- 汇报与交接文档：`PROJECT_PRESENTATION_GUIDE.md`
- 边缘部署说明：`ops/edge/README.md`

## 9. 已知边界

1. 当前为流程级仿真，不是高保真刚体动力学仿真。  
2. 视觉与IO仍为模拟模式，真机接入在下一阶段。  
3. 前端采用单文件静态页，不引入构建工具。

