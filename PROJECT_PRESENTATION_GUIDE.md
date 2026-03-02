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

## 6.8 代码深度批注（可直接当讲稿）

> 这一节是“逐段代码背后的控制语义”，不是重复代码表面意思，而是解释每一行在工业控制链路中的职责。

### A) `WorkcellLayout` 为什么要放在后端

- `dirty_rack_pick / clean_rack_place_base / washer_infeed / return_pick`：这是流程节点的“控制真值坐标”，决定机器人真实应该去哪里。  
- `dirty_observe / return_observe`：观察位（非操作位），用于让视觉识别与抓放动作解耦，避免“边看边抓”导致的状态震荡。  
- `safe_home`：安全过渡点，核心作用是“相邻动作解耦”。很多碰撞/抖动并不出在抓放动作本身，而出在动作切换瞬间。  
- 这组点位在后端统一，意味着控制层只有一个坐标权威源（single source of truth），前端只负责渲染映射。

### B) `WORKCELL_LAYOUT` 为什么还要在前端再定义一份

- 前端这份不是“控制命令”，而是“可视化几何布局参数”（包括清洗机实体、传送带尺寸、回流环半径）。  
- `washerBody / conveyorUpper / conveyorLower / returnLoop / returnChute` 本质是场景搭建参数，不是抓取目标点。  
- 需要与后端关键工位保持一致，是因为你在做的是“数字孪生可视化”，不是“艺术化动画”；视觉错位会让调试结论失真。  
- 所以两份配置看似重复，实则一份管控制、另一份管表达，职责分离但坐标要对齐。

### C) `_build_side_pick_segments` 的工程语义

- `reset_seed(self._joint_angles_rad)`：把当前关节作为下一次 IK 初值，减少求解跳支路。  
- `approach_sign`：根据目标在机器人左右两侧选择进给方向，避免统一从单侧接近导致不可达。  
- `q = _get_dynamic_quat(...)`：动态决定末端姿态，保证“侧吸方向”与工件法向关系一致。  
- `pre/contact/lift/retreat`：典型工业动作模板。  
  - `pre`：进入前的安全缓冲。  
  - `contact`：与工件接触/建立吸附。  
  - `lift`：先抬离，避免横向拖拽碰撞。  
  - `retreat`：沿安全方向退出。  
- `on_vacuum_on -> tray.bind_to_tool(...)`：控制层吸附成功后才绑定，避免“视觉先吸上、物理未吸上”的假状态。  
- `lock_orientation=True`：侧吸任务对姿态极其敏感，锁姿态比“自由最优”更稳定。

### D) `_build_place_to_washer_segments` 的工程语义

- `target_tray_center.z = infeed.z + tray_half_thickness`：把几何厚度显式纳入放置目标，防止穿透地面/台面。  
- `contact + pre + retreat + home`：放置也走完整轨迹，不做“瞬移落点”，保证过程可解释、可回放、可诊断。  
- `on_release()` 中的状态切换很关键：  
  - `vacuum.turn_off()`：执行器状态变化。  
  - `tray.release_from_tool(...)`：托盘所有权从工具坐标系切回世界/产线坐标系。  
  - `self._washer_tray = tray`：移交给清洗线子系统接管。  
  - `self._active_tray = None`：主抓取流程结束当前抓取上下文。  
- 这是一个标准“控制权移交（handoff）”流程，避免同一对象被两个子系统同时驱动。

### E) `ik_solver.py` 为什么能缓解“手腕抽搐”

- `self._last_solution`：保存上一帧 IK 解，形成时间连续性。  
- `reset_seed(current_joints)`：将链路关节映射回 IK 全量链向量，确保“求解器内部状态”和“机器人当前状态”一致。  
- `orientation_mode = "all"`：对末端 3 轴姿态全约束，减少“位置到达但姿态翻腕”的等价解抖动。  
- `return IKResult(True/False, joints)`：把“是否可达”提升为显式信号，便于上层做降级/重试策略，而不是吞异常。

### F) `setTrayVisualAttachment`（前端）为什么要改父子关系

- `ee.add(tray)`：吸附后托盘坐标变成末端局部坐标，天然随动。  
- `scene.attach(tray)`：释放时保持世界位姿不变地切回根节点，避免视觉跳变。  
- 这比“每帧手动复制世界坐标”更稳定，因为复用了 Three.js 的层级变换机制。  
- `approachSign` 计算托盘贴附在工具左右哪一侧，保证视觉位置与真实侧吸方向一致。

### G) `updateScene` 为什么必须以后端真值为准

- `backendBoundToTool` 是控制层事实，不是前端猜测。  
- 吸附时忽略 `tray_body.pose`，避免两套位姿源同时写同一对象（经典竞争条件）。  
- 未吸附时才 `lerp(trayGoal)`，吸附时完全走父子级联。  
- 这个判定本质是在做“单写者原则（single writer）”：任一时刻只有一个模块有权更新托盘位姿。

## 6.9 专业术语批注（汇报可直接解释）

- `URDF`：机器人结构描述文件，定义连杆、关节、关节限位和网格资源；是“机械结构数字化底座”。  
- `IK (Inverse Kinematics)`：逆运动学，输入末端目标位姿，输出关节角。你这里是“位姿驱动关节”，不是反过来。  
- `TCP (Tool Center Point)`：工具中心点，吸盘等末端工具的参考点；偏移错 1~2 cm 就会表现为抓取缝隙。  
- `FSM (Finite State Machine)`：有限状态机，把流程拆成可控状态和转移条件，避免“巨型 if-else 泥团”。  
- `Trajectory Segment`：轨迹分段，每段有起终点、速度、姿态约束、触发回调，是运动控制可解释性的核心。  
- `Seed`（IK 初值）：求解器的起点，影响收敛到哪个解分支；连续 seed 可以显著减少抖动。  
- `Quaternion`（四元数）：姿态表示方法，避免欧拉角万向锁；适合连续旋转约束。  
- `lock_orientation`：对轨迹段施加姿态锁定，防止执行中末端扭腕。  
- `WebSocket`：前后端实时状态通道，用于 100ms 级状态推送。  
- `Gateway`：协议与边界层，做 REST/WS 代理、命令串行化、降级保护，隔离前端与执行服务。  
- `Degraded Mode`（降级模式）：后端异常时系统保持可观测、可恢复，而不是整站崩溃。  
- `Handoff`（控制权移交）：对象从一个控制器（机械臂）转交到另一个控制器（清洗线/输送），防止双重驱动。

## 6.10 为什么这么做（设计动机，可直接放答辩）

我们采用“分层架构 + 状态机分段 + IK 连续约束 + 单一真值源”的设计，不是为了把代码写复杂，而是为了把工业流程的三个核心目标同时满足：**稳定性、可解释性、可迁移性**。稳定性来自于姿态约束、seed 连续和单写者原则；可解释性来自于分段轨迹与显式状态流，任何异常都能定位到具体状态和段；可迁移性来自 Gateway/Adapter 边界，后续替换仿真适配器即可接入 ROS2/FANUC/真实 IO，而不需要推翻前端和流程控制主干。

## 6.11 全代码块逐行批注（语法 + 含义 + 原因）

> 说明：下面按“代码块 -> 行号（L1/L2...）”逐行解释。每行都给出三件事：**语法结构**、**这行是什么意思**、**为什么这么写**。

### 6.11.1 架构图代码块（`text`）

代码块位置：第 3 节。

- `L1 Browser (frontend/index.html)`：语法=普通文本节点；含义=前端入口；原因=先讲用户可见层。  
- `L2 ├─ 3D Scene (Three.js + URDFLoader)`：语法=树形子节点；含义=3D 渲染模块；原因=强调几何可视化职责。  
- `L3 ├─ Dashboard UI (状态、日志、按钮)`：语法=树形并列子节点；含义=面板交互层；原因=区分“显示控制”与“运动控制”。  
- `L4 └─ WS /ws + REST /api/*`：语法=终端子节点；含义=通信协议出口；原因=把前端对外依赖明确化。  
- `L5 │`：语法=树形连接符；含义=上下层连接；原因=强化分层流向。  
- `L6 ▼`：语法=方向符号；含义=请求/状态向下传递；原因=让架构方向直观。  
- `L7 Node Gateway (gateway/src/index.ts)`：语法=第二层根节点；含义=网关服务；原因=隔离前端和执行层。  
- `L8 ├─ API代理 /api/* -> Python /internal/*`：语法=映射描述；含义=REST 转发；原因=统一对外 API。  
- `L9 ├─ WS桥接 /ws <-> /internal/ws`：语法=双向箭头；含义=双向实时桥接；原因=保留流式能力。  
- `L10 ├─ 命令串行化队列（防并发冲突）`：语法=说明补语；含义=命令按序执行；原因=避免多命令竞态。  
- `L11 └─ Python断线降级状态`：语法=终端子节点；含义=后端异常兜底；原因=提升演示稳定性。  
- `L12 │`：语法=连接符；含义=继续下钻层级；原因=视觉一致。  
- `L13 ▼`：语法=方向符号；含义=进入执行层；原因=体现控制链路。  
- `L14 Python Execution Service (backend/app/main.py)`：语法=第三层根节点；含义=执行服务入口；原因=标识控制核心。  
- `L15 ├─ SidePickExecutionFSM (side_pick_runtime.py)`：语法=组件列举；含义=运行时封装；原因=解耦前端协议。  
- `L16 ├─ VisionGuidedSidePickFSM (vision_side_pick_fsm.py)`：语法=组件列举；含义=流程状态机；原因=承载业务动作逻辑。  
- `L17 ├─ IK Solver (ik_solver.py, ikpy)`：语法=组件列举；含义=逆解模块；原因=把目标位姿转换为关节角。  
- `L18 ├─ Sim Adapters (adapters.py / simulator.py)`：语法=组件列举；含义=仿真适配层；原因=未来可替换真机驱动。  
- `L19 └─ SystemState WS广播 (models.py + ws_manager.py)`：语法=终端组件；含义=状态协议与广播；原因=给前端唯一状态源。  

### 6.11.2 Docker 启动代码块（`bash`）

代码块位置：第 4.1 节。

- `L1 docker compose up -d --build`：语法=命令 + 子命令 + 参数；含义=构建并后台启动服务；原因=一条命令降低上手成本。  
  - `-d`：语法=短参数；含义=detached 后台运行；原因=终端不阻塞。  
  - `--build`：语法=长参数；含义=启动前重建镜像；原因=确保代码变更生效。  

### 6.11.3 `WorkcellLayout` 代码块（`python`）

- `L1 @dataclass`：语法=装饰器；含义=自动生成初始化/表示方法；原因=减少模板代码、突出数据本体。  
- `L2 class WorkcellLayout:`：语法=类定义；含义=工位坐标容器；原因=集中管理布局常量。  
- `L3 dirty_rack_pick: Vec3 = field(...)`：语法=类型注解 + 默认工厂；含义=脏盘抓取点；原因=避免可变默认值陷阱并保留语义字段。  
- `L4 clean_rack_place_base: Vec3 = field(...)`：语法同上；含义=净盘架放置基准点；原因=抓放对称定义便于维护。  
- `L5 washer_infeed: Vec3 = field(...)`：语法同上；含义=清洗线入口点；原因=放置目标统一引用。  
- `L6 return_pick: Vec3 = field(...)`：语法同上；含义=回流抓取点；原因=回流阶段使用固定锚点。  
- `L7 dirty_observe: Vec3 = field(...)`：语法同上；含义=脏盘观察位；原因=视觉动作与抓取动作解耦。  
- `L8 return_observe: Vec3 = field(...)`：语法同上；含义=回流观察位；原因=流程一致性。  
- `L9 safe_home: Vec3 = field(...)`：语法同上；含义=安全中间位；原因=跨阶段过渡更稳。  

### 6.11.4 `WORKCELL_LAYOUT` 代码块（`javascript`）

- `L1 const WORKCELL_LAYOUT = {`：语法=常量对象字面量开始；含义=前端场景布局配置；原因=集中管理可视化坐标。  
- `L2 dirtyRack: { x_m:..., y_m:..., z_m:... },`：语法=对象嵌套；含义=脏盘架位置；原因=与后端关键点对齐。  
- `L3 cleanRack: {...},`：语法同上；含义=净盘架位置；原因=渲染与流程一致。  
- `L4 washerLoad: {...},`：语法同上；含义=清洗线入料位；原因=可视化抓放目标。  
- `L5 washerReturn: {...},`：语法同上；含义=回流出料位；原因=定义回流路径节点。  
- `L6 washerBody: {...},`：语法=结构参数对象；含义=清洗机实体尺寸与中心；原因=构建场景几何体。  
- `L7 conveyorUpper: {...},`：语法同上；含义=上层传送带尺寸；原因=表达物料上行路径。  
- `L8 conveyorLower: {...},`：语法同上；含义=下层传送带尺寸；原因=表达回流路径。  
- `L9 returnLoop: {...},`：语法同上；含义=回流环路径几何；原因=让流线可见。  
- `L10 returnChute: {...}`：语法同上（末尾无逗号）；含义=回流槽体；原因=补全回流连接。  
- `L11 };`：语法=对象结束；含义=配置常量闭合；原因=供后续模块统一读取。  

### 6.11.5 `_build_side_pick_segments` 代码块（`python`）

- `L1 def ... -> list[TrajectorySegment]:`：语法=函数定义 + 返回类型注解；含义=生成侧吸轨迹段；原因=把动作规划独立为纯构建函数。  
- `L2 if self._ik_solver is not None and hasattr(...):`：语法=复合条件；含义=检查 IK 与能力；原因=兼容不同求解器实现。  
- `L3 self._ik_solver.reset_seed(...)`：语法=方法调用；含义=重置 IK 初值；原因=跨段解连续。  
- `L4 approach_sign = ...`：语法=赋值；含义=决定接近方向符号；原因=统一左右侧策略。  
- `L5 q = ...`：语法=赋值；含义=计算目标姿态四元数；原因=侧吸姿态稳定。  
- `L6 standoff = ...`：语法=赋值；含义=取预接近距离；原因=防止直接撞入目标。  
- `L7 contact = Vec3(...)`：语法=对象构造；含义=接触点；原因=定义吸附发生位置。  
- `L8 pre = Vec3(...)`：语法=对象构造；含义=预抓点；原因=形成缓冲段。  
- `L9 lift = Vec3(...)`：语法=对象构造；含义=抬升点；原因=避免横向摩擦/碰撞。  
- `L10 retreat = Vec3(...)`：语法=对象构造；含义=撤退点；原因=离开危险区域。  
- `L11 start_pose = self._ee_pose.copy()`：语法=拷贝调用；含义=当前起点姿态；原因=避免原位引用被后续修改。  
- `L12 pre_pose = self._solve_ik...`：语法=函数调用；含义=预抓点 IK 解；原因=位姿转关节角。  
- `L13 contact_pose = ...`：语法同上；含义=接触点 IK 解；原因=保证可执行。  
- `L14 lift_pose = ...`：语法同上；含义=抬升点 IK 解；原因=形成连续动作链。  
- `L15 retreat_pose = ...`：语法同上；含义=撤退点 IK 解；原因=动作完整闭环。  
- `L16 tray_side_offset = Vec3(...)`：语法=对象构造；含义=托盘相对工具侧偏移；原因=吸附后托盘几何对齐。  
- `L17 def on_vacuum_on() -> None:`：语法=内部函数定义；含义=段触发回调；原因=把副作用绑定到时间点。  
- `L18 self.vacuum.turn_on()`：语法=方法调用；含义=打开真空；原因=执行抓取动作。  
- `L19 if self.vacuum.vacuum_ok:`：语法=条件分支；含义=仅成功吸附才继续；原因=防止假抓取。  
- `L20 tray.bind_to_tool(...)`：语法=方法调用；含义=绑定托盘到工具坐标；原因=控制权切换清晰。  
- `L21 return [`：语法=返回列表开始；含义=返回轨迹序列；原因=交由执行器统一消费。  
- `L22 self._linear_segment("pre_grasp", ...)`：语法=函数调用项；含义=预抓直线段；原因=安全靠近。  
- `L23 self._horizontal_push_segment("contact_push", ...)`：语法=函数调用项；含义=水平接触推进；原因=模拟侧向贴靠。  
- `L24 TrajectorySegment("vacuum_on_dwell", ...)`：语法=对象实例化；含义=停留并触发吸附；原因=给真空建立时间。  
- `L25 self._linear_segment("retreat_lift_2cm", ...)`：语法=函数调用项；含义=抬升撤离段；原因=先脱离接触面。  
- `L26 self._horizontal_push_segment("retreat_back_15cm", ...)`：语法=函数调用项；含义=水平后退；原因=回到安全通道。  
- `L27 ]`：语法=列表结束；含义=轨迹构建完成；原因=函数输出固定结构。  

### 6.11.6 `_build_place_to_washer_segments` 代码块（`python`）

- `L1 def ... -> list[TrajectorySegment]:`：语法=函数定义；含义=生成放置到清洗线轨迹；原因=抓与放分开便于测试。  
- `L2 if self._ik_solver ...:`：语法=条件判断；含义=能力检查；原因=兼容性。  
- `L3 self._ik_solver.reset_seed(...)`：语法=调用；含义=重置初值；原因=减小分段抖动。  
- `L4 infeed = self.layout.washer_infeed`：语法=赋值；含义=读取入口基准；原因=避免硬编码重复。  
- `L5 target_tray_center = Vec3(...)`：语法=对象构造；含义=托盘中心放置目标；原因=加入半厚度防穿模。  
- `L6 approach_sign = ...`：语法=赋值；含义=进给方向；原因=适应左右工位。  
- `L7 q = ...`：语法=赋值；含义=放置姿态；原因=保持姿态一致。  
- `L8 contact = Vec3(`：语法=多行构造开始；含义=接触放置点；原因=可读性提高。  
- `L9 target_tray_center.x + ...`：语法=表达式；含义=按半宽修正 x；原因=侧向接触几何对齐。  
- `L10 target_tray_center.y,`：语法=参数；含义=y 保持；原因=只沿必要轴调整。  
- `L11 target_tray_center.z,`：语法=参数；含义=z 使用中心高；原因=避免高度漂移。  
- `L12 )`：语法=构造结束；含义=接触点定义完成；原因=供后续段复用。  
- `L13 pre = Vec3(...)`：语法=构造；含义=接近点；原因=插入安全缓冲。  
- `L14 retreat = Vec3(...)`：语法=构造；含义=撤退点；原因=释放后离开工位。  
- `L15 home = self.layout.safe_home`：语法=赋值；含义=收尾安全位；原因=流程归一。  
- `L16 start_pose = self._ee_pose.copy()`：语法=拷贝；含义=当前姿态；原因=连续规划。  
- `L17 pre_pose = ...`：语法=调用；含义=pre IK；原因=执行落地。  
- `L18 place_pose = ...`：语法=调用；含义=place IK；原因=目标姿态可达。  
- `L19 retreat_pose = ...`：语法=调用；含义=retreat IK；原因=回撤可执行。  
- `L20 home_pose = ...`：语法=调用；含义=home IK；原因=动作闭环。  
- `L21 def on_release() -> None:`：语法=内部函数；含义=释放时回调；原因=把状态修改集中。  
- `L22 self.vacuum.turn_off()`：语法=调用；含义=关真空；原因=物理释放。  
- `L23 tray.is_clean = False`：语法=属性赋值；含义=标记未清洗（等待流程）；原因=状态机语义一致。  
- `L24 exact_infeed_pose = self._make_pose(...)`：语法=调用赋值；含义=构造精确接管姿态；原因=减少接管误差。  
- `L25 tray.release_from_tool(exact_infeed_pose)`：语法=调用；含义=解除工具绑定；原因=控制权移交。  
- `L26 self._washer_tray = tray`：语法=赋值；含义=托盘交给清洗模块；原因=子系统边界清晰。  
- `L27 self._active_tray = None`：语法=赋值；含义=清空当前抓取对象；原因=避免双持有。  
- `L28 self._washer_elapsed_s = 0.0`：语法=赋值；含义=重置清洗计时；原因=下阶段计时起点明确。  
- `L29 return [`：语法=返回列表；含义=输出动作段；原因=统一执行接口。  
- `L30 self._linear_segment("washer_approach", ...)`：语法=列表项；含义=接近段；原因=安全进入。  
- `L31 self._horizontal_push_segment("washer_insert", ...)`：语法=列表项；含义=插入段；原因=贴合清洗入口动作。  
- `L32 TrajectorySegment("vacuum_off_release", ...)`：语法=实例化；含义=释放停留段；原因=留释放稳定时间。  
- `L33 self._horizontal_push_segment("washer_retreat", ...)`：语法=列表项；含义=后撤段；原因=避免停在危险区域。  
- `L34 self._linear_segment("washer_clear", ...)`：语法=列表项；含义=清场回家段；原因=为下一任务准备。  
- `L35 ]`：语法=结束；含义=轨迹构建完成；原因=固定输出。  

### 6.11.7 `reset_seed` 代码块（`python`）

- `L1 def reset_seed(...) -> None:`：语法=函数定义；含义=重置 IK 初值；原因=给下一次求解连续起点。  
- `L2 if current_joints is None:`：语法=条件；含义=无输入关节；原因=支持冷启动。  
- `L3 self._last_solution = np.zeros(...)`：语法=库函数调用；含义=全零初始解；原因=提供合法缺省向量。  
- `L4 return`：语法=提前返回；含义=结束函数；原因=减少嵌套。  
- `L5 full = np.array(..., copy=True)`：语法=数组拷贝；含义=复制上次解；原因=避免原地污染。  
- `L6 if full.shape[0] != len(self._chain.links):`：语法=尺寸检查；含义=维度是否匹配；原因=防链配置变化导致越界。  
- `L7 full = np.zeros(...)`：语法=重建数组；含义=维度不匹配时兜底；原因=保证求解器稳定。  
- `L8 joints = [float(v) for v in current_joints]`：语法=列表推导；含义=统一转 float；原因=输入规范化。  
- `L9 for j_idx, chain_idx in enumerate(...):`：语法=枚举循环；含义=遍历关节映射表；原因=业务关节到链索引映射。  
- `L10 if j_idx < len(joints):`：语法=边界保护；含义=防输入长度不足；原因=容错。  
- `L11 full[chain_idx] = joints[j_idx]`：语法=索引赋值；含义=写入对应关节值；原因=形成完整初值向量。  
- `L12 self._last_solution = full`：语法=属性赋值；含义=保存新 seed；原因=后续求解复用。  

### 6.11.8 `solve_tcp_pose` 代码块（`python`）

- `L1 def solve_tcp_pose(... ) -> IKResult:`：语法=函数定义+返回注解；含义=解 TCP 到关节；原因=统一 IK 调用入口。  
- `L2 target_position = np.array([...], dtype=float)`：语法=数组构造；含义=构造目标位置向量；原因=匹配 ikpy 输入。  
- `L3 kwargs = {...}`：语法=字典字面量；含义=基础求解参数；原因=后续可复用扩展。  
- `L4 if side_pick_quat_xyzw is not None and len(...) == 4:`：语法=复合条件；含义=是否提供姿态约束；原因=同一接口兼容两种模式。  
- `L5 # 四元数 -> 旋转矩阵`：语法=注释；含义=说明转换步骤；原因=提高可读性。  
- `L6 ...`：语法=省略占位；含义=中间转换逻辑未展示；原因=文档聚焦关键结构。  
- `L7 orient_kwargs = dict(kwargs)`：语法=浅拷贝字典；含义=复制基础参数；原因=不污染原 `kwargs`。  
- `L8 orient_kwargs["target_orientation"] = rot`：语法=键赋值；含义=加入目标姿态矩阵；原因=启用姿态约束。  
- `L9 orient_kwargs["orientation_mode"] = "all"`：语法=键赋值；含义=全轴约束；原因=抑制姿态跳变。  
- `L10 solution = self._chain.inverse_kinematics(**orient_kwargs)`：语法=关键字参数解包；含义=带姿态求解；原因=严格控制末端朝向。  
- `L11 else:`：语法=分支；含义=无姿态约束路径；原因=支持一般位移任务。  
- `L12 solution = self._chain.inverse_kinematics(**kwargs)`：语法=函数调用；含义=仅位置求解；原因=更灵活且计算更轻。  
- `L13 full = np.asarray(solution, dtype=float)`：语法=数组转换；含义=标准化结果；原因=后续索引安全。  
- `L14 self._last_solution = full`：语法=赋值；含义=缓存当前解；原因=下一次可作 seed。  
- `L15 joints = [float(full[idx]) for idx in self._joint_indices]`：语法=列表推导；含义=提取有效关节；原因=屏蔽无效链节点。  
- `L16 return IKResult(True, joints)`：语法=对象返回；含义=成功结果；原因=上层统一处理协议。  

### 6.11.9 `setTrayVisualAttachment` 代码块（`javascript`）

- `L1 const setTrayVisualAttachment = (attachToTool) => {`：语法=箭头函数；含义=设置托盘是否挂到工具；原因=封装附着逻辑。  
- `L2 const shouldAttach = !!attachToTool;`：语法=双重取反；含义=强制布尔化；原因=规避非布尔输入。  
- `L3 if (shouldAttach === trayAttachedVisual) return;`：语法=早返回；含义=状态没变就不做事；原因=减少重复计算与抖动。  
- `L4 scene.updateMatrixWorld(true);`：语法=方法调用；含义=刷新场景世界矩阵；原因=确保后续位姿读取准确。  
- `L5 ee.updateMatrixWorld(true);`：语法=方法调用；含义=刷新末端世界矩阵；原因=附件切换前先同步变换。  
- `L6 if (shouldAttach) {`：语法=条件分支；含义=进入吸附流程；原因=区分 attach/detach。  
- `L7 ee.add(tray);`：语法=父子挂接；含义=托盘成为末端子节点；原因=利用层级变换自动跟随。  
- `L8 const eeWorld = new THREE.Vector3();`：语法=对象实例化；含义=临时世界坐标变量；原因=计算侧向。  
- `L9 ee.getWorldPosition(eeWorld);`：语法=方法调用；含义=读取末端世界位置；原因=判定左右侧。  
- `L10 const isRightSide = eeWorld.x > 0;`：语法=比较表达式；含义=是否在右侧；原因=决定偏移方向。  
- `L11 const approachSign = isRightSide ? -1 : 1;`：语法=三元运算；含义=方向符号；原因=让托盘贴附到正确侧边。  
- `L12 tray.position.set(...);`：语法=向量赋值；含义=设置托盘局部偏移；原因=吸附几何对齐。  
- `L13 tray.rotation.set(0, 0, 0);`：语法=旋转赋值；含义=重置局部转角；原因=避免继承旧姿态误差。  
- `L14 } else {`：语法=分支切换；含义=进入释放流程；原因=双向操作完整。  
- `L15 scene.attach(tray);`：语法=保持世界位姿重挂；含义=托盘回到场景根；原因=释放时不跳变。  
- `L16 }`：语法=分支结束；含义=attach/detach 逻辑完结；原因=结构闭合。  
- `L17 trayAttachedVisual = shouldAttach;`：语法=状态写回；含义=记录当前可视附着状态；原因=供下次去重。  
- `L18 };`：语法=函数结束；含义=导出可调用逻辑；原因=在 `updateScene` 中复用。  

### 6.11.10 `updateScene` 判定代码块（`javascript`）

- `L1 const backendBoundToTool = !!backendTrayBody.bound_to_tool;`：语法=布尔归一；含义=后端吸附真值；原因=单一事实来源。  
- `L2 ...`：语法=省略；含义=中间逻辑未展开；原因=聚焦关键判定。  
- `L3 trayAttached = backendBoundToTool;`：语法=赋值；含义=前端吸附状态跟随后端；原因=避免前端自作主张。  
- `L4 ...`：语法=省略；含义=其它更新流程；原因=文档节选。  
- `L5 const ignoreBackendTrayPose = trayAttachedVisual || backendBoundToTool;`：语法=逻辑或；含义=判断是否屏蔽绝对位姿；原因=吸附期避免双写。  
- `L6 if (hasBackendTrayPose && !ignoreBackendTrayPose) {`：语法=复合条件；含义=未吸附且有后端位姿时更新目标；原因=正常跟随后台。  
- `L7 trayGoal.set(...);`：语法=向量设置；含义=目标位姿设为后端绝对位姿；原因=保持同步。  
- `L8 } else if (hasBackendTrayPose) {`：语法=分支；含义=有位姿但处于吸附期；原因=避免冲突。  
- `L9 trayGoal.copy(trayVisualPos);`：语法=向量复制；含义=目标锁到当前视觉位姿；原因=防止被后端绝对位姿抢控制。  
- `L10 }`：语法=分支结束；含义=位姿来源判定结束；原因=进入后续渲染步骤。  
- `L11 ...`：语法=省略；含义=中间逻辑；原因=节选。  
- `L12 setTrayVisualAttachment(tray.visible && trayAttached);`：语法=函数调用+逻辑与；含义=按可见性与吸附状态决定挂接；原因=状态一致。  
- `L13 if (trayAttachedVisual && trayAttached) {`：语法=复合条件；含义=确认处于吸附态；原因=选择随动路径。  
- `L14 tray.getWorldPosition(trayWorldTmp);`：语法=调用；含义=取托盘世界位姿；原因=给视觉缓存同步。  
- `L15 trayVisualPos.copy(trayWorldTmp);`：语法=复制；含义=视觉位姿直接跟父子级联；原因=彻底避免漂移。  
- `L16 } else {`：语法=分支；含义=未吸附路径；原因=切回插值跟随。  
- `L17 trayVisualPos.lerp(trayGoal, trayFollowAlpha);`：语法=线性插值；含义=平滑趋近目标；原因=去抖和自然动画。  
- `L18 setTrayWorldPosition(trayVisualPos);`：语法=调用；含义=写回托盘世界坐标；原因=更新渲染对象。  
- `L19 }`：语法=分支结束；含义=本帧托盘更新完成；原因=流程闭合。  

### 6.11.11 发布流程代码块（`powershell`）

- `L1 # 1) 本地打包`：语法=注释；含义=步骤说明；原因=避免误操作。  
- `L2 cd D:\robert_arm_sensor`：语法=切目录命令；含义=进入项目根目录；原因=保证相对路径正确。  
- `L3 tar -czf ...`：语法=命令 + 选项 + 文件列表；含义=生成压缩包。  
  - `-c`：创建新归档。  
  - `-z`：gzip 压缩。  
  - `-f`：后接输出文件名。  
  - 后续文件列表：明确只打包部署需要的目录与配置。  
- `L4 (空行)`：语法=分隔；含义=分开步骤；原因=可读性。  
- `L5 # 2) 上传`：语法=注释；含义=第二步说明；原因=流程清晰。  
- `L6 scp D:\...\tar.gz ubuntu@117.72.52.43:~/`：语法=远程拷贝命令；含义=把部署包传到服务器家目录；原因=与云端解压步骤衔接。  

### 6.11.12 云端部署代码块（`bash`）

- `L1 # 3) 云端重部署`：语法=注释；含义=步骤标题；原因=区分本地与云端操作。  
- `L2 rm -rf ~/robert_arm_sensor_new`：语法=删除命令；含义=清旧目录；原因=避免脏文件残留。  
- `L3 mkdir -p ~/robert_arm_sensor_new`：语法=创建目录；含义=创建目标目录；原因=幂等执行。  
- `L4 tar -xzf ~/robert_arm_sensor_deploy.tar.gz -C ~/robert_arm_sensor_new`：语法=解压命令；含义=解包到目标目录；原因=部署内容标准化。  
- `L5 (空行)`：语法=分隔；含义=进入服务操作段；原因=阅读友好。  
- `L6 cd ~/robert_arm_sensor_new`：语法=切目录；含义=进入部署目录；原因=后续 compose 使用正确上下文。  
- `L7 docker-compose down --remove-orphans`：语法=停止命令；含义=停止并移除孤儿容器；原因=清理旧运行态。  
- `L8 docker-compose build --no-cache backend gateway`：语法=构建命令；含义=无缓存重建指定服务；原因=确保镜像非陈旧层。  
- `L9 docker-compose up -d`：语法=启动命令；含义=后台拉起服务；原因=部署完成后驻留运行。  
- `L10 docker-compose ps`：语法=查询命令；含义=查看服务状态；原因=立即验证部署是否成功。  

### 6.11.13 这一套“逐行写法”背后的统一逻辑

- 逻辑主线：**先定义权威状态 -> 再规划分段动作 -> 再做执行/渲染同步 -> 最后做异常与部署闭环**。  
- 语法风格：Python 侧重“类型注解 + 小函数 + 回调挂钩”；JavaScript 侧重“对象配置 + 状态判定 + 场景层级变换”；Shell 侧重“步骤化、可复现”。  
- 这样写的核心原因：把复杂工业流程拆成可验证的小单元，每行代码都对应一个明确职责，便于调试、答辩、交接和后续真机迁移。

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

