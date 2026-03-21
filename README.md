# AutonomousDrivingDemoWASM

Ported from the original Python repo at [FredBill1/AutonomousDrivingDemo](https://github.com/FredBill1/AutonomousDrivingDemo) via vibe coding. I don't understand a single line of code in this repo.

<details>

<summary>AGENTS.md I used</summary>

```markdown
我正在用Vite + Rust WASM将使用PySide6编写的`AutonomousDrivingDemo`文件夹中的项目(启动方式为`python -m AutonomousDrivingDemo`)移植到`web`文件夹中，希望最终能在GitHub Pages上运行。现在`web`文件夹中有一个基本的骨架，但是存在非常多的问题需要解决。

目标是最终移植后的程序的行为和原本的Python实现**完全一致**，仅在UI上有所区别。

需要注意的是，不要自己编造新的逻辑，一切以Python实现为准。当前`web`的实现有很多**不一致**的地方，需要找出并**删除**它们。

每个步骤的修改完成后，需要在`web`文件夹中运行npm编译、测试，并进行git commit（`web`文件夹里包含独立于当前根目录的git仓库）。

后续工作必须遵守以下事项：

1. Python 实现是唯一真值。任何 web / Rust / WASM 行为如果无法在以下文件中找到明确依据，就视为不应该存在：
   - `AutonomousDrivingDemo/MainWindow.py`
   - `AutonomousDrivingDemo/MapServerNode.py`
   - `AutonomousDrivingDemo/CarSimulationNode.py`
   - `AutonomousDrivingDemo/GlobalPlannerNode.py`
   - `AutonomousDrivingDemo/LocalPlannerNode.py`
   - `AutonomousDrivingDemo/TrajectoryCollisionCheckingNode.py`
   - `AutonomousDrivingDemo/modeling/Car.py`
   - `AutonomousDrivingDemo/modeling/Obstacles.py`
   - `AutonomousDrivingDemo/global_planner/hybrid_a_star.py`
   - `AutonomousDrivingDemo/local_planner/ModelPredictiveControl.py`
   - `C:/ProgramData/miniconda3/envs/py314/Lib/site-packages/rsplan/*.py`

2. 删除错误逻辑，补齐缺失逻辑。当前 `web` 中已经确认存在 fabricated logic，需要删除

3. 迁移时必须按 Python 节点职责对齐，而不是按前端方便程度自由重组逻辑。至少要能清晰对应这些职责：
   - MainWindow orchestration
   - MapServer
   - CarSimulation
   - GlobalPlanner
   - LocalPlanner
   - TrajectoryCollisionChecking

4. 必须严格保持以下关键行为一致：
   - 车辆状态原点是后轴中心，不是车体中心。
   - 激光扫描圆心位于 `rear axle + BACK_TO_CENTER`。
   - 碰撞检测使用 `Car.check_collision` 等价逻辑，而不是近似距离判断。
   - Global planner 使用 Python 的参数、代价模型、Reeds-Shepp analytic expansion 语义和失败语义。
   - Local planner 必须是 Python 等价的 stateful MPC。
   - 仿真必须执行“带时间戳的 target velocity / steer 序列插值”。
   - Replanning 时必须保留 Python 的 brake trajectory 起步逻辑。

5. 地图与障碍系统必须以 Python 当前行为为准：
   - unknown obstacles 的数量、生成方式、发现方式都必须与 `MapServerNode.py` 一致。
   - 随机初始位姿必须使用真实碰撞模型筛选。
   - `web` 中的地图可以是手动hard coded的，暂时无需改为从图片读取。

6. UI 可以不同，但语义不能不同。尤其要对齐：
   - `Set Goal / Set Pose / Brake / Cancel / Restart` 的状态机。
   - 鼠标拖拽设定 pose/goal 的方式。
   - 快捷键 `A/S/D/F/R`。
   - Visualization / Velocity / Steer 三类显示内容。
   - global trajectory / local trajectory / reference points / explored segments / unreachable text 的显示条件。
   - 出于网页端性能考虑，Hybrid A*算法在执行时的已探索节点只显示最近的32个batch，而不是像 Python 实现中的显示全部已探索节点。

7. `web` 中使用PixiJS进行绘图。为了适配浏览器操作，鼠标和触控的交互逻辑与Python实现略有不同：
   - 桌面端使用鼠标操作时：
     - 鼠标左键拖拽行为保持不变，即set pose/goal。
     - 鼠标中键拖拽为移动viewport
     - 鼠标滚轮为缩放viewport
   - 移动端使用触控操作时：
     - 单指拖拽为set pose/goal
     - 双指拖拽为移动/缩放viewport

8. 每个独立步骤完成后，必须在 `web` 目录执行并确认通过：
   - `npm run build`
   - `npm run check:wasm`
   - 然后进行 git commit

9.  每次提交前必须自查：
   - 本次改动是否能明确对应到某个 Python 文件中的真实行为。
   - 是否删除了至少一个已确认不一致的 web 逻辑，补齐了至少一个确实缺失的 Python 行为。
   - 是否引入了新的 UI 文案、状态变量、fallback 逻辑、控制逻辑；若有，原则上应删除。
   - 若发现 Python 与 web 行为不一致，默认选择 Python，不做折中。
```

</details>
