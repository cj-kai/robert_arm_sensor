# Edge Deployment Notes (Phase-1)

本目录用于记录“边缘工控机/现场 PC”部署方案（真机接入阶段）。

## 1. 推荐运行拆分

1. ROS2 + FANUC Driver：先运行在主机（不强制容器化）  
2. Python Execution Service：主机或容器均可  
3. Node Gateway：主机或容器均可  
4. 前端统一从 Gateway 访问（只对外开放 Gateway 端口）

## 2. 组件职责

- `gateway/`：对前端提供稳定 API/WS，做代理、降级、命令串行化  
- `backend/`：运行 FSM、IK、适配器（视觉/机器人/真空/输送）  
- `ROS2/FANUC`：真机控制与状态回读（下一阶段）

## 3. 现场部署建议

1. 首次联调先用主机直跑，缩短排障路径。  
2. 真机稳定后再考虑容器化与 systemd 守护。  
3. 生产环境将实时控制网络与办公网络隔离。  
4. 保留急停、围栏和低速调试策略。

## 4. 预留 systemd 服务名（建议）

- `crx-gateway.service`
- `crx-exec.service`
- `crx-ros-driver.service`

