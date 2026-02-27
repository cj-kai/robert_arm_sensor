# CRX 托盘清洗仿真平台讲解文档（Presentation 全流程版）

> 目标读者：对项目完全不了解的计算机初学者。  
> 使用场景：给导师、项目资助人做汇报；写项目经历；后续团队交接。

---

## 1. 项目一句话介绍

这是一个“网页可视化 + 状态机流程 + 机械臂运动学”的工业托盘搬运仿真系统。  
系统模拟了完整流程：**识别脏托盘 -> 侧吸抓取 -> 放到清洗线 -> 清洗回流 -> 再抓取 -> 放回净盘架**，并支持故障注入与恢复。

---

## 2. 项目价值（给导师/资助人先讲这段）

### 2.1 业务价值

1. 在没有真机和复杂产线的情况下，先把“流程逻辑 + 可视化 + 控制接口”跑通。  
2. 能快速验证工位布局、节拍逻辑、异常恢复策略，降低后期真机联调成本。  
3. 保留真实接入路径（ROS2/FANUC、USB 相机、IO），不是一次性 Demo。

### 2.2 技术价值

1. 前后端分层清晰：`Frontend(Three.js)` + `Gateway(Node)` + `Execution(Python FastAPI)`。  
2. 支持 URDF 工业机械臂模型加载与关节驱动。  
3. 支持 WebSocket 实时状态流和故障降级（后端断线前端不崩）。

---

## 3. 系统架构图（汇报可直接贴）

```text
Browser (frontend/index.html)
  ├─ 3D Scene (Three.js + URDFLoader)
  ├─ Dashboard UI (状态、日志、按钮)
  └─ WS /ws + REST /api/*
            │
            ▼
Node Gateway (gateway/src/index.ts)
  ├─ API代理 /api/* -> Python /internal/*
  ├─ WS桥接 /ws <-> /internal/ws
  ├─ 命令串行化队列（防并发冲突）
  └─ Python断线降级状态
            │
            ▼
Python Execution Service (backend/app/main.py)
  ├─ SidePickExecutionFSM (side_pick_runtime.py)
  ├─ VisionGuidedSidePickFSM (vision_side_pick_fsm.py)
  ├─ IK Solver (ik_solver.py, ikpy)
  ├─ Sim Adapters (adapters.py / simulator.py)
  └─ SystemState WS广播 (models.py + ws_manager.py)
```

---

## 4. 从零搭建到可运行（给小白照着做）

## 4.1 最简单启动（Docker）

```bash
docker compose up -d --build
```

打开浏览器：

- `http://127.0.0.1:8000/`（本地）
- 或 `http://<服务器IP>:8000/`

## 4.2 页面上的按钮是什么意思

1. `Start`：启动一轮托盘流程。  
2. `Stop`：停止当前任务。  
3. `Reset`：复位状态机和计数。  
4. `Next Pre-Suction Fail`：注入“下一次预吸失败”。  
5. `Drop Once During Transport`：注入“运输中掉落”。

## 4.3 数据如何流动

1. 前端点击按钮 -> `POST /api/...` 到 Gateway。  
2. Gateway 代理到 Python `/internal/...`。  
3. Python 更新 FSM 状态，每 100ms 广播一次 WS。  
4. 前端 `updateScene(data)` 用这份状态更新 3D 和面板。

---

## 5. 项目文件全量说明（逐个文件）

下面按目录逐个说明“这个文件干什么、是否建议改动”。

## 5.1 根目录

| 文件 | 作用 | 是否常改 |
|---|---|---|
| `.cursor/worktrees.json` | Cursor 工作区元数据，和业务逻辑无关 | 否 |
| `.dockerignore` | Docker 构建时忽略文件规则 | 偶尔 |
| `.gitignore` | Git 忽略规则 | 偶尔 |
| `docker-compose.yml` | 一键启动 `backend + gateway` 服务编排 | 是 |
| `README.md` | 项目入口说明（运行、架构、接口） | 是 |
| `PROJECT_学习与排查手册.md` | 过程性学习/排查手册（历史文档） | 可选 |
| `PROJECT_PRESENTATION_GUIDE.md` | 本文档（汇报与交接） | 是 |
| `robert_arm_sensor_deploy.tar.gz` | 离线部署包（上传云端用） | 每次发布重打 |

## 5.2 backend 目录

| 文件 | 作用 | 是否常改 |
|---|---|---|
| `backend/Dockerfile` | Python 镜像构建：装依赖、拷代码、拷前端静态文件 | 偶尔 |
| `backend/requirements.txt` | Python 依赖：FastAPI、uvicorn、numpy、ikpy 等 | 偶尔 |
| `backend/app/__init__.py` | 包初始化文件 | 否 |
| `backend/app/main.py` | FastAPI 入口，REST + WS + MJPEG 接口，广播调度 | 是 |
| `backend/app/models.py` | `SystemState` 数据结构定义（WS 协议核心） | 是 |
| `backend/app/ws_manager.py` | WS 连接管理与广播 | 偶尔 |
| `backend/app/config_store.py` | JSON 配置读写与默认参数合并 | 是 |
| `backend/app/adapters.py` | 适配器接口与模拟实现（Robot/Vision/Vacuum/Conveyor） | 是 |
| `backend/app/simulator.py` | 旧版模拟器基础组件（VisionSim/RobotSim/VacuumSim等） | 偶尔 |
| `backend/app/fsm.py` | 旧版 `GraspFSM`（兼容历史逻辑） | 低频 |
| `backend/app/side_pick_runtime.py` | 新核心执行封装：把新 FSM 映射成前端可消费状态 | 是 |
| `backend/app/vision_side_pick_fsm.py` | 新核心状态机：工位布局、轨迹规划、抓放流程 | 高频 |
| `backend/app/ik_solver.py` | IK 求解（ikpy），输出 `joint_angles_rad` | 高频 |

## 5.3 gateway 目录

| 文件 | 作用 | 是否常改 |
|---|---|---|
| `gateway/Dockerfile` | Gateway 镜像构建，运行 Node 服务 | 偶尔 |
| `gateway/package.json` | Node 依赖与脚本 | 偶尔 |
| `gateway/package-lock.json` | 依赖锁定文件 | 自动生成 |
| `gateway/tsconfig.json` | TypeScript 编译配置 | 偶尔 |
| `gateway/src/index.ts` | 网关核心：REST代理、WS桥接、降级、命令队列 | 高频 |

## 5.4 frontend 目录

| 文件 | 作用 | 是否常改 |
|---|---|---|
| `frontend/index.html` | 前端唯一主文件：UI + 3D + WS + 控制逻辑 | 高频 |
| `frontend/vendor/three/three.module.js` | Three.js 主库（第三方） | 不改 |
| `frontend/vendor/three/OrbitControls.js` | Three.js 轨道控制器（第三方） | 不改 |
| `frontend/vendor/three/URDFLoader.js` | URDF 加载器（第三方） | 不改 |
| `frontend/vendor/three/URDFClasses.js` | URDF 类定义（第三方） | 不改 |
| `frontend/vendor/three/ColladaLoader.js` | DAE 网格加载器（第三方） | 不改 |
| `frontend/vendor/three/TGALoader.js` | TGA 纹理加载器（第三方） | 不改 |
| `frontend/vendor/chartjs/chart.umd.min.js` | 图表库（第三方） | 不改 |
| `frontend/vendor/tailwind/tailwind-playcdn.js` | Tailwind CDN 脚本（第三方） | 不改 |
| `frontend/assets/robot/crx20ial/robot.urdf` | CRX-20 URDF 模型定义（关节/连杆/轴/限位） | 是 |
| `frontend/assets/robot/crx20ial/meshes/README.txt` | mesh 资源说明 | 可选 |
| `frontend/assets/robot/crx20ial/meshes/crx20ial/visual/*.dae` | FANUC 视觉网格（base,j1~j6） | 不改 |
| `frontend/assets/robot/crx20ial/meshes/crx20ial/collision/*.stl` | FANUC 碰撞网格（base,j1~j6） | 不改 |

## 5.5 tools / ops 目录

| 文件 | 作用 | 是否常改 |
|---|---|---|
| `tools/robot_assets/fetch_fanuc_crx20_assets.ps1` | 从 FANUC 开源仓库下载 CRX-20 mesh 资源 | 偶尔 |
| `ops/edge/README.md` | 边缘机部署说明占位（真机阶段） | 可选 |

---

## 6. 核心代码讲解（带注释版）

这里是你在汇报里最值得讲的“技术实质”。

## 6.1 后端工位布局（绝对坐标）

文件：`backend/app/vision_side_pick_fsm.py`

```python
@dataclass
class WorkcellLayout:
    dirty_rack_pick: Vec3 = field(default_factory=lambda: Vec3(-0.75, 0.75, 0.80))
    clean_rack_place_base: Vec3 = field(default_factory=lambda: Vec3(-0.75, -0.75, 0.80))
    washer_infeed: Vec3 = field(default_factory=lambda: Vec3(0.80, 0.60, 0.50))
    return_pick: Vec3 = field(default_factory=lambda: Vec3(0.80, -0.60, 0.50))
    dirty_observe: Vec3 = field(default_factory=lambda: Vec3(-0.85, 0.55, 0.95))
    return_observe: Vec3 = field(default_factory=lambda: Vec3(0.65, -0.75, 0.95))
    safe_home: Vec3 = field(default_factory=lambda: Vec3(0.40, 0.0, 0.90))
```

解释：

1. 所有流程点位都在这里统一管理。  
2. 这一层是“后端真实控制参考坐标”。  
3. 前端 `WORKCELL_LAYOUT` 必须与这里一致，否则就会出现“机器人去A点，托盘画在B点”的错位。

## 6.2 前端工位布局（可视化坐标）

文件：`frontend/index.html`

```javascript
const WORKCELL_LAYOUT = {
    dirtyRack: { x_m: -0.75, y_m: 0.75, z_m: 0.0 },
    cleanRack: { x_m: -0.75, y_m: -0.75, z_m: 0.0 },
    washerLoad: { x_m: 0.80, y_m: 0.60, z_m: 0.5 },
    washerReturn: { x_m: 0.80, y_m: -0.60, z_m: 0.5 },
    washerBody: { center_x_m: 2.65, center_y_m: 0.0, size_x_m: 1.35, size_y_m: 0.72, size_h_m: 0.60 },
    conveyorUpper: { center_x_m: 1.725, center_y_m: 0.60, size_x_m: 2.00, size_y_m: 0.18, size_h_m: 0.06 },
    conveyorLower: { center_x_m: 1.725, center_y_m: -0.60, size_x_m: 2.00, size_y_m: 0.18, size_h_m: 0.06 },
    returnLoop: { center_x_m: 2.65, center_y_m: 0.0, radius_m: 0.60, tube_m: 0.05 },
    returnChute: { center_x_m: 1.65, center_y_m: -0.60, size_x_m: 1.00, size_y_m: 0.16, size_h_m: 0.05 }
};
```

解释：

1. 这是“画面布局坐标”。  
2. 如果它和后端 `WorkcellLayout` 不一致，会直接导致瞬移/穿模/抓空。  
3. 项目后期把传送带拉长，是为了让空间关系更接近真实工位。

## 6.3 抓取轨迹规划（侧吸）

文件：`backend/app/vision_side_pick_fsm.py`

```python
def _build_side_pick_segments(self, target: Vec3, tray: SimTray) -> list[TrajectorySegment]:
    if self._ik_solver is not None and hasattr(self._ik_solver, "reset_seed"):
        self._ik_solver.reset_seed(self._joint_angles_rad)

    approach_sign = self._approach_sign_for_target(target)
    q = self._get_dynamic_quat(approach_sign)   # 动态偏航翻转
    standoff = self._approach_standoff_m()

    contact = Vec3(target.x, target.y, target.z)                # 直接贴侧面
    pre = Vec3(contact.x + approach_sign * standoff, contact.y, contact.z)
    lift = Vec3(contact.x, contact.y, contact.z + self.cfg.retreat_lift_m)
    retreat = Vec3(pre.x, pre.y, lift.z)

    start_pose = self._ee_pose.copy()
    pre_pose = self._solve_ik_with_locked_orientation(pre, q)
    contact_pose = self._solve_ik_with_locked_orientation(contact, q)
    lift_pose = self._solve_ik_with_locked_orientation(lift, q)
    retreat_pose = self._solve_ik_with_locked_orientation(retreat, q)

    tray_side_offset = Vec3(-approach_sign * (tray.dims_m.x / 2.0), 0.0, 0.0)

    def on_vacuum_on() -> None:
        self.vacuum.turn_on()
        if self.vacuum.vacuum_ok:
            tray.bind_to_tool(self._ee_pose, tray_side_offset)

    return [
        self._linear_segment("pre_grasp", start_pose, pre_pose, lock_orientation=True),
        self._horizontal_push_segment("contact_push", pre_pose, contact_pose, speed_scale=self.cfg.contact_push_speed_scale),
        TrajectorySegment("vacuum_on_dwell", contact_pose, contact_pose, self.cfg.contact_dwell_s, lock_orientation=True, on_start=on_vacuum_on),
        self._linear_segment("retreat_lift_2cm", contact_pose, lift_pose, lock_orientation=True),
        self._horizontal_push_segment("retreat_back_15cm", lift_pose, retreat_pose, speed_scale=self.cfg.retreat_push_speed_scale),
    ]
```

解释：

1. `pre -> contact -> dwell -> lift -> retreat` 是标准工业抓取分段。  
2. `on_vacuum_on` 时绑定托盘到工具坐标。  
3. `lock_orientation=True` 保证侧吸阶段不乱翻腕。

## 6.4 放入清洗线轨迹

文件：`backend/app/vision_side_pick_fsm.py`

```python
def _build_place_to_washer_segments(self, tray: SimTray) -> list[TrajectorySegment]:
    if self._ik_solver is not None and hasattr(self._ik_solver, "reset_seed"):
        self._ik_solver.reset_seed(self._joint_angles_rad)

    infeed = self.layout.washer_infeed
    target_tray_center = Vec3(infeed.x, infeed.y, infeed.z + tray.dims_m.z / 2.0)  # 带半厚度
    approach_sign = self._approach_sign_for_target(target_tray_center)
    q = self._get_dynamic_quat(approach_sign)

    contact = Vec3(
        target_tray_center.x + approach_sign * (tray.dims_m.x / 2.0),
        target_tray_center.y,
        target_tray_center.z,
    )
    pre = Vec3(contact.x + approach_sign * 0.20, contact.y, contact.z)
    retreat = Vec3(contact.x + approach_sign * 0.20, contact.y, contact.z + 0.03)
    home = self.layout.safe_home

    start_pose = self._ee_pose.copy()
    pre_pose = self._solve_ik_with_locked_orientation(pre, q)
    place_pose = self._solve_ik_with_locked_orientation(contact, q)
    retreat_pose = self._solve_ik_with_locked_orientation(retreat, q)
    home_pose = self._solve_ik_with_locked_orientation(home, q)

    def on_release() -> None:
        self.vacuum.turn_off()
        tray.is_clean = False
        exact_infeed_pose = self._make_pose(target_tray_center, q)
        tray.release_from_tool(exact_infeed_pose)
        self._washer_tray = tray
        self._active_tray = None
        self._washer_elapsed_s = 0.0

    return [
        self._linear_segment("washer_approach", start_pose, pre_pose, lock_orientation=True),
        self._horizontal_push_segment("washer_insert", pre_pose, place_pose, speed_scale=self.cfg.contact_push_speed_scale),
        TrajectorySegment("vacuum_off_release", place_pose, place_pose, self.cfg.release_dwell_s, lock_orientation=True, on_start=on_release),
        self._horizontal_push_segment("washer_retreat", place_pose, retreat_pose, speed_scale=self.cfg.retreat_push_speed_scale),
        self._linear_segment("washer_clear", retreat_pose, home_pose, lock_orientation=True),
    ]
```

解释：

1. `target_tray_center.z + half_thickness` 解决“释放时穿模/下坠瞬移”。  
2. 放置和抓取一样用分段，不做瞬时跳点。  
3. `on_release` 把托盘切到“清洗线接管”状态。

## 6.5 IK 求解器（为什么不再乱扭）

文件：`backend/app/ik_solver.py`

```python
def reset_seed(self, current_joints: Sequence[float] | None = None) -> None:
    if current_joints is None:
        self._last_solution = np.zeros(len(self._chain.links), dtype=float)
        return
    full = np.array(self._last_solution, dtype=float, copy=True)
    if full.shape[0] != len(self._chain.links):
        full = np.zeros(len(self._chain.links), dtype=float)
    joints = [float(v) for v in current_joints]
    for j_idx, chain_idx in enumerate(self._joint_indices):
        if j_idx < len(joints):
            full[chain_idx] = joints[j_idx]
    self._last_solution = full
```

```python
def solve_tcp_pose(self, xyz_m, side_pick_quat_xyzw=None) -> IKResult:
    target_position = np.array([float(xyz_m[0]), float(xyz_m[1]), float(xyz_m[2])], dtype=float)
    kwargs = {"target_position": target_position, "initial_position": self._last_solution}

    if side_pick_quat_xyzw is not None and len(side_pick_quat_xyzw) == 4:
        # 四元数 -> 旋转矩阵
        ...
        orient_kwargs = dict(kwargs)
        orient_kwargs["target_orientation"] = rot
        orient_kwargs["orientation_mode"] = "all"  # 严格全轴姿态约束
        solution = self._chain.inverse_kinematics(**orient_kwargs)
    else:
        solution = self._chain.inverse_kinematics(**kwargs)

    full = np.asarray(solution, dtype=float)
    self._last_solution = full
    joints = [float(full[idx]) for idx in self._joint_indices]
    return IKResult(True, joints)
```

解释：

1. `reset_seed(current_joints)` 用当前姿态作为下一段 IK 初值，减少跨段翻腕。  
2. `orientation_mode="all"` 强约束末端姿态，防止手腕“看起来到位但姿态乱翻”。  
3. 求解失败会返回 `IKResult(False, ...)`，由上层决定如何处理。

## 6.6 前端托盘吸附渲染（父子级联）

文件：`frontend/index.html`

```javascript
const setTrayVisualAttachment = (attachToTool) => {
    const shouldAttach = !!attachToTool;
    if (shouldAttach === trayAttachedVisual) return;
    scene.updateMatrixWorld(true);
    ee.updateMatrixWorld(true);
    if (shouldAttach) {
        ee.add(tray); // 变成工具子节点
        const eeWorld = new THREE.Vector3();
        ee.getWorldPosition(eeWorld);
        const isRightSide = eeWorld.x > 0;
        const approachSign = isRightSide ? -1 : 1;
        tray.position.set(-approachSign * (trayDimsCurrent.x / 2.0), 0, 0);
        tray.rotation.set(0, 0, 0);
    } else {
        scene.attach(tray); // 释放时回到场景根节点
    }
    trayAttachedVisual = shouldAttach;
};
```

解释：

1. “吸住”不是改托盘世界坐标，而是改变父子关系。  
2. 这样托盘会跟随末端运动，避免单独动画导致漂移。  
3. 放下时 `scene.attach(tray)` 保持世界姿态连续。

## 6.7 前端 updateScene 的关键判定（是否吸附谁说了算）

文件：`frontend/index.html`

```javascript
const backendBoundToTool = !!backendTrayBody.bound_to_tool;
...
trayAttached = backendBoundToTool; // 后端真值唯一来源
...
const ignoreBackendTrayPose = trayAttachedVisual || backendBoundToTool;
if (hasBackendTrayPose && !ignoreBackendTrayPose) {
    trayGoal.set(...); // 未吸附时按后端绝对位姿走
} else if (hasBackendTrayPose) {
    trayGoal.copy(trayVisualPos); // 吸附时不再吃绝对位姿，防冲突
}
...
setTrayVisualAttachment(tray.visible && trayAttached);
if (trayAttachedVisual && trayAttached) {
    tray.getWorldPosition(trayWorldTmp);
    trayVisualPos.copy(trayWorldTmp); // 完全由父子级联驱动
} else {
    trayVisualPos.lerp(trayGoal, trayFollowAlpha); // 仅未吸附时插值
    setTrayWorldPosition(trayVisualPos);
}
```

解释：

1. 后端 `bound_to_tool` 是唯一吸附真值。  
2. 吸附期间屏蔽 `tray_body.pose` 绝对位姿，避免“幽灵抢夺”冲突。  
3. 未吸附才允许 `lerp`。

---

## 7. 为什么之前会出现“抓取缝隙 / 手腕锁死 / 放置瞬移”

这是你汇报里最能体现工程能力的“问题复盘”。

## 7.1 抓取缝隙

常见根因：

1. 前后端 `tcp_offset` 不一致。  
2. 吸附边沿时托盘未先对齐，直接切父子关系导致把误差锁死。  
3. 吸附状态仍被绝对位姿更新覆盖。

处理：

1. 对齐偏移与工具坐标定义。  
2. 只在未吸附状态使用绝对位姿插值。  
3. 吸附后完全走父子变换。

## 7.2 手腕锁死/抽搐

常见根因：

1. IK 无姿态约束，或约束过弱导致分支跳跃。  
2. IK 初值继承错误，每段都从不合理 seed 重新解。  
3. 工位点位超出舒适工作空间。

处理：

1. 引入四元数姿态约束（`orientation_mode`）。  
2. `reset_seed(current_joints)` 保持求解连续性。  
3. 收缩工位布局到可达域。

## 7.3 放置瞬移

常见根因：

1. 放置高度没加托盘半厚度。  
2. 释放点与后续输送接管起点坐标不一致。  
3. 前端释放瞬间被 fallback 目标拉回。

处理：

1. 放置目标中心高度 = 平面高度 + 托盘半厚度。  
2. `release_from_tool` 与输送起点统一。  
3. 放置状态下强制托盘保持 `placeGoal`。

---

## 8. 演示 PPT 建议（10 页模板）

1. **封面**：项目名 + 目标（托盘清洗流程数字孪生）。  
2. **业务场景**：脏盘 -> 清洗 -> 净盘回架。  
3. **架构图**：Frontend / Gateway / Backend。  
4. **数据流**：按钮命令、WS状态流、3D渲染更新。  
5. **URDF 与 IK**：为什么要有 `joint_angles_rad`。  
6. **FSM 轨迹**：pre/contact/release/retreat。  
7. **问题复盘**：缝隙、锁死、瞬移（根因+修复）。  
8. **当前效果**：实时状态、日志、故障注入。  
9. **可扩展路线**：真实 ROS2/相机/IO 接入。  
10. **总结**：阶段成果 + 下一阶段计划。

---

## 9. Demo 讲解词（可直接念）

“这个系统把工业托盘搬运拆成三层：前端可视化、网关、执行层状态机。  
前端并不直接控制机器人，而是消费后端状态。后端每 100ms 输出一次机器人位姿、关节角、托盘状态。  
机械臂模型来自 URDF，关节角由 IK 求解器计算，避免了纯动画假动作。  
我们重点解决了三类工程问题：  
第一，坐标系和工位布局不一致导致抓取错位；  
第二，吸附过程中父子关系和绝对位姿冲突导致托盘漂移；  
第三，IK 种子与姿态约束不稳定导致手腕抽搐。  
目前系统已支持完整流程仿真、故障注入、日志追踪，并为真机接入保留接口。”

---

## 10. 简历项目经历写法（中英文）

## 10.1 中文版（可直接贴）

- 主导开发 `Three.js + Node.js + FastAPI` 的工业机械臂托盘清洗流程仿真平台，实现“识别-抓取-搬运-放置-恢复”全链路闭环。  
- 基于 URDF + IK（ikpy）实现 6 轴关节实时驱动，输出 `joint_angles_rad` 同步前端 3D 机械臂姿态。  
- 设计并落地状态机（FSM）分段轨迹（pre-grasp/contact/release/retreat），完成侧吸抓取与输送接管逻辑。  
- 构建 Gateway 层实现 REST/WS 代理、命令串行化、后端断线降级，提升演示稳定性和可维护性。  
- 通过坐标系统一、吸附父子级联重构、IK 姿态约束与种子连续性优化，系统性解决抓取缝隙、手腕抽搐与放置瞬移问题。

## 10.2 英文版（可直接贴）

- Built an industrial tray-washing simulation platform with `Three.js + Node.js + FastAPI`, covering end-to-end flow: detection, pick, transfer, place, and recovery.  
- Integrated URDF-based 6-DOF robot visualization with IK-driven joint outputs (`joint_angles_rad`) for real-time kinematic consistency.  
- Implemented an FSM with segmented Cartesian trajectories (pre-grasp/contact/release/retreat) for side-pick and conveyor handoff logic.  
- Developed a gateway layer for REST/WS proxying, command serialization, and degraded-state fallback when execution service is unavailable.  
- Resolved major motion/rendering issues (grasp gap, wrist twitch, placement teleport) via coordinate alignment, strict attachment ownership, and IK seed/orientation stabilization.

---

## 11. 可能被问到的问题（答辩准备）

### Q1：为什么不用 Gazebo / Isaac Sim？

A：本阶段目标是流程闭环和控制架构验证，不是高保真动力学。Three.js + FSM + IK 在开发速度和演示稳定性上更优，且保留了真机接入边界。

### Q2：这个系统能接真机吗？

A：能。Gateway/Execution 的接口边界已固定，后续替换 `SimAdapter` 为 ROS2/FANUC 适配器即可，不需要推翻前端和网关层。

### Q3：为什么会出现“看起来像瞬移”？

A：通常是三类冲突：坐标系不一致、吸附期间姿态来源冲突、放置接管点不一致。我们已经在代码层分别加了约束与统一。

### Q4：下一步最关键是什么？

A：真实相机标定（内参+手眼）、真实机器人状态回读、真实 IO 闭环（真空/输送带），再做异常恢复策略验证。

---

## 12. 固定发布流程（本地 -> 云端）

```powershell
# 1) 本地打包
cd D:\robert_arm_sensor
tar -czf robert_arm_sensor_deploy.tar.gz backend frontend gateway ops tools docker-compose.yml README.md .dockerignore .gitignore

# 2) 上传
scp D:\robert_arm_sensor\robert_arm_sensor_deploy.tar.gz ubuntu@117.72.52.43:~/
```

```bash
# 3) 云端重部署
rm -rf ~/robert_arm_sensor_new
mkdir -p ~/robert_arm_sensor_new
tar -xzf ~/robert_arm_sensor_deploy.tar.gz -C ~/robert_arm_sensor_new

cd ~/robert_arm_sensor_new
docker-compose down --remove-orphans
docker-compose build --no-cache backend gateway
docker-compose up -d
docker-compose ps
```

---

## 13. 结语

这不是“做了个网页动画”，而是一个具备工业控制迁移路径的分层仿真平台。  
你在汇报时只要讲清楚三件事就足够打动导师和资助人：

1. 我们把复杂流程拆成了可维护的三层架构。  
2. 我们不仅实现了功能，还解决了工程级联调问题。  
3. 这套系统可以平滑升级到真机与真实视觉/IO。

