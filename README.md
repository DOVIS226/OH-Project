# MuJoCo 卧室场景说明文档

## 项目简介
这是一个基于 MuJoCo 物理引擎构建的简单卧室仿真场景。场景中包含完整的室内家具布置，并集成了一个 6 自由度机械臂和移动小车，可用于机器人操作、导航等研究实验。

## 文件结构
```
mujoco_bedroom/
├── bedroom_scene.xml          # 主场景文件
├── README.md                  # 本说明文档
├── test_scene.py             # Python测试脚本
├── assets/                   # 模型资源文件夹（预留）
└── textures/                 # 纹理资源文件夹（预留）
```

## 场景描述

### 整体布局
- **房间尺寸**: 10m × 8m × 5m（长×宽×高）
- **环境**: 带有地板、三面墙壁的卧室空间
- **光照**: 双光源设置，提供自然的照明效果

### 关键物体列表

#### 1. 环境基础设施
| 物体名称 | 描述 | 位置 | 尺寸 |
|---------|------|------|------|
| floor | 木质纹理地板 | (0, 0, 0) | 10m × 8m |
| wall_back | 后墙 | (0, -4, 2.5) | 10m × 0.2m × 5m |
| wall_left | 左墙 | (-5, 0, 2.5) | 0.2m × 8m × 5m |
| wall_right | 右墙 | (5, 0, 2.5) | 0.2m × 8m × 5m |

#### 2. 家具物品
| 物体名称 | 描述 | 位置 | 主要组件 |
|---------|------|------|----------|
| bed | 单人床 | (-3, -2.5, 0) | 床垫、床架、床头板 |
| desk | 书桌 | (3, -2, 0) | 桌面、4条桌腿 |
| chair | 椅子 | (3, -0.8, 0) | 座椅、椅背、4条椅腿 |
| wardrobe | 衣柜 | (3.5, -3.2, 0) | 柜体、左右柜门 |
| nightstand | 床头柜 | (-1.3, -2.5, 0) | 柜面、柜体 |

#### 3. 装饰物品
| 物体名称 | 描述 | 位置 |
|---------|------|------|
| lamp | 台灯 | (-1.3, -2.5, 0.58) 在床头柜上 |
| books | 书本堆叠（3本） | (2.5, -2, 0.78) 在书桌上 |

#### 4. 机器人设备

##### A. 6自由度机械臂
- **名称**: robot_base
- **位置**: 放置在书桌上 (3, -2, 0.78)
- **自由度**: 6个旋转/俯仰关节
- **末端执行器**: 双指夹爪（可开合）
- **关节列表**:
  - joint1: 基座旋转（±180°）
  - joint2: 肩关节（±90°）
  - joint3: 肘关节（±150°）
  - joint4: 腕关节1（±180°）
  - joint5: 腕关节2（±120°）
  - joint6: 腕关节3（±180°）
  - left_finger_joint: 左夹爪（0-0.03m）
  - right_finger_joint: 右夹爪（0-0.03m）

##### B. 移动小车
- **名称**: mobile_robot
- **位置**: 房间中央 (0, 1, 0.1)
- **自由度**: 6自由度（可自由移动和旋转）
- **组件**: 车体、4个轮子、传感器平台
- **质量**: 5kg

## 技术特性

### 1. 物理模拟
- **时间步长**: 0.005秒
- **重力**: 标准重力加速度 (0, 0, -9.81)
- **摩擦系数**: 地板配置真实摩擦参数

### 2. 视觉效果
- **纹理**: 使用 MuJoCo 内置纹理生成器
  - 地板: 棋盘格木质纹理
  - 墙壁: 平面浅色纹理
  - 家具: 木质纹理
  - 金属: 金属光泽材质
- **光照**: 多光源配置，包含高光和环境光
- **阴影**: 启用高质量阴影渲染（2048分辨率）

### 3. 控制接口
场景提供了 8 个电机驱动器：
- 6个机械臂关节电机（motor1-motor6）
- 2个夹爪电机（gripper_left, gripper_right）

## 使用方法

### 安装依赖
```bash
pip install mujoco
pip install numpy
```

### 快速开始

#### 1. 基础运行
```bash
# 使用默认设置运行（带演示动画）
python test_scene.py

# 跳过演示动画
python test_scene.py --no-demo

# 以30 FPS运行（降低性能要求）
python test_scene.py --fps 30

# 从特定相机视角启动
python test_scene.py --camera robot_view

# 仅显示场景信息
python test_scene.py --info

# 显示详细调试信息
python test_scene.py --verbose
```

#### 2. 在Python中加载场景
```python
import mujoco
import mujoco.viewer

# 加载模型
model = mujoco.MjModel.from_xml_path('bedroom_scene.xml')
data = mujoco.MjData(model)

# 启动查看器
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 控制机械臂示例

#### 基础位置控制
```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path('bedroom_scene.xml')
data = mujoco.MjData(model)

# 设置关节目标位置（弧度）
target_positions = [0.5, 0.3, -0.4, 0.2, 0.1, 0.0]

# 在仿真循环中应用控制
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 设置前6个执行器（机械臂关节）
        for i in range(6):
            data.ctrl[i] = target_positions[i]

        # 设置夹爪（0-0.5，0表示闭合）
        data.ctrl[6] = 0.3  # 左夹爪
        data.ctrl[7] = 0.3  # 右夹爪

        mujoco.mj_step(model, data)
        viewer.sync()
```

#### 读取关节状态
```python
# 获取关节位置
joint_positions = data.qpos[:6]  # 前6个关节
print(f"关节位置: {joint_positions}")

# 获取关节速度
joint_velocities = data.qvel[:6]
print(f"关节速度: {joint_velocities}")

# 获取传感器数据
sensor_data = data.sensordata
print(f"传感器数据: {sensor_data}")

# 获取末端执行器位置
ee_site_id = model.site('ee_site').id
ee_position = data.site_xpos[ee_site_id]
print(f"末端执行器位置: {ee_position}")
```

#### 逆运动学示例
```python
import mujoco
import numpy as np

def move_to_position(model, data, target_pos, steps=1000):
    """
    使用简单的雅可比逆运动学将末端移动到目标位置

    Args:
        model: MuJoCo模型
        data: MuJoCo数据
        target_pos: 目标位置 [x, y, z]
        steps: 最大迭代步数
    """
    ee_site_id = model.site('ee_site').id

    for _ in range(steps):
        # 获取当前末端位置
        current_pos = data.site_xpos[ee_site_id].copy()

        # 计算位置误差
        error = target_pos - current_pos

        if np.linalg.norm(error) < 0.01:  # 到达目标
            break

        # 计算雅可比矩阵
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, ee_site_id)

        # 使用伪逆计算关节速度
        J_pinv = np.linalg.pinv(jacp[:, :6])
        dq = J_pinv @ (error * 0.1)  # 比例控制

        # 应用控制
        data.ctrl[:6] = data.qpos[:6] + dq

        # 执行仿真步
        mujoco.mj_step(model, data)

# 使用示例
model = mujoco.MjModel.from_xml_path('bedroom_scene.xml')
data = mujoco.MjData(model)

# 移动到目标位置
target = np.array([3.2, -1.8, 1.0])
move_to_position(model, data, target)
```

### 传感器数据访问

```python
# 获取所有传感器名称
for i in range(model.nsensor):
    sensor_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
    print(f"传感器 {i}: {sensor_name}")

# 读取特定传感器数据
joint1_pos_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'joint1_pos')
joint1_position = data.sensordata[joint1_pos_sensor_id]
print(f"关节1位置: {joint1_position}")

# 读取力/力矩传感器
force_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'ee_force')
ee_force = data.sensordata[force_sensor_id:force_sensor_id+3]
print(f"末端执行器力: {ee_force}")
```

## 自定义与扩展

### 添加新物体
在 `<worldbody>` 标签中添加新的 `<body>` 元素：
```xml
<body name="new_object" pos="x y z">
  <geom name="new_geom" type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
</body>
```

### 更换纹理
1. 将自定义纹理文件放入 `textures/` 文件夹
2. 在 `<asset>` 中定义纹理：
```xml
<texture name="custom_tex" type="2d" file="textures/your_texture.png"/>
<material name="custom_mat" texture="custom_tex"/>
```

### 添加其他机器人模型
可以将外部机器人模型（如 UR5, Franka Panda）通过 `<include>` 引入：
```xml
<include file="path/to/robot_model.xml"/>
```

## 坐标系统
- **X轴**: 房间宽度方向（左-右）
- **Y轴**: 房间深度方向（前-后）
- **Z轴**: 房间高度方向（上）
- **原点**: 房间地板中心

## 性能优化建议
1. **降低时间步长**：默认 timestep 为 0.002s，如果不需要高精度，可以增加到 0.005s
2. **调整渲染质量**：修改 `<visual><quality shadowsize="1024"/>` 降低阴影分辨率
3. **禁用不需要的物体**：注释掉装饰物体可以提高性能
4. **降低求解器迭代次数**：将 `iterations` 从 50 降至 20-30
5. **使用较低的 FPS**：运行 `python test_scene.py --fps 30` 而不是默认的 60

## 故障排除

### 常见问题

#### 1. 导入错误：找不到 mujoco 模块
```
ModuleNotFoundError: No module named 'mujoco'
```
**解决方案**：
```bash
pip install mujoco --upgrade
```

#### 2. 场景加载失败
```
Error: XML Error: ... at line X
```
**解决方案**：
- 检查 XML 文件格式是否正确
- 确保所有引用的纹理和资源存在
- 验证关节范围和物理参数合理性

#### 3. 仿真不稳定或爆炸
**症状**：物体飞出场景、速度异常、关节抖动

**解决方案**：
- 增加求解器迭代次数（`iterations`）
- 降低时间步长（`timestep`）
- 检查质量和惯性参数是否合理
- 调整接触参数（`solimp`, `solref`）

```xml
<!-- 在 <option> 标签中 -->
<option timestep="0.001" iterations="100" tolerance="1e-10"/>
```

#### 4. 渲染性能问题
**症状**：帧率过低、卡顿

**解决方案**：
```bash
# 降低目标帧率
python test_scene.py --fps 30

# 或在代码中调整
```

```xml
<!-- 降低渲染质量 -->
<visual>
  <quality shadowsize="512"/>
  <global offwidth="1280" offheight="720"/>
</visual>
```

#### 5. 机械臂控制不响应
**解决方案**：
- 检查执行器索引是否正确
- 确保控制值在 `ctrlrange` 范围内
- 验证关节没有被锁定或卡住

```python
# 检查执行器信息
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    ctrl_range = model.actuator_ctrlrange[i]
    print(f"执行器 {i}: {name}, 范围: {ctrl_range}")
```

#### 6. 夹爪无法抓取物体
**原因**：摩擦系数不足或接触参数不当

**解决方案**：
- 增加夹爪几何体的摩擦系数
- 调整接触刚度和阻尼
- 确保物体和夹爪的 contype/conaffinity 设置允许接触

```xml
<!-- 在夹爪几何体上 -->
<geom ... friction="1.5 0.01 0.0001" solimp="0.99 0.99 0.001"/>
```

### 调试技巧

#### 1. 启用详细日志
```bash
python test_scene.py --verbose
```

#### 2. 可视化接触力
```python
# 在viewer中启用接触力可视化
# 按 Ctrl+F 或在GUI中启用 "Contact Forces"
```

#### 3. 打印物理诊断信息
```python
# 检查是否有接触
print(f"接触数量: {data.ncon}")

# 检查约束违反
print(f"约束残差: {np.linalg.norm(data.efc_force)}")

# 检查能量
print(f"动能: {mujoco.mj_energyKin(model, data)}")
print(f"势能: {mujoco.mj_energyPot(model, data)}")
```

#### 4. 录制仿真
```python
import mujoco
import mediapy as media

model = mujoco.MjModel.from_xml_path('bedroom_scene.xml')
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, 640, 480)

frames = []
for _ in range(300):
    mujoco.mj_step(model, data)
    renderer.update_scene(data)
    pixels = renderer.render()
    frames.append(pixels)

media.write_video('simulation.mp4', frames, fps=60)
```

## API 参考

### 关键数据结构

#### MjModel - 模型静态信息
```python
model.nq          # 广义坐标数量
model.nv          # 自由度数量
model.nu          # 执行器数量
model.nbody       # 刚体数量
model.njnt        # 关节数量
model.ngeom       # 几何体数量
model.nsensor     # 传感器数量
model.opt         # 仿真选项
```

#### MjData - 仿真动态数据
```python
data.time         # 当前仿真时间
data.qpos         # 广义坐标（位置）
data.qvel         # 广义速度
data.ctrl         # 控制输入
data.qacc         # 广义加速度
data.qfrc_applied # 外部施加的力
data.sensordata   # 传感器读数
data.xpos         # 笛卡尔位置
data.xquat        # 笛卡尔四元数
```

### 常用函数

#### 仿真步进
```python
mujoco.mj_step(model, data)              # 完整仿真步
mujoco.mj_forward(model, data)           # 仅正向动力学
mujoco.mj_inverse(model, data)           # 逆向动力学
```

#### 雅可比计算
```python
jacp = np.zeros((3, model.nv))
jacr = np.zeros((3, model.nv))
mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
```

#### ID转换
```python
# 名称转ID
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot_base')

# ID转名称
body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
```

### 场景配置参数说明

| 参数 | 位置 | 说明 | 推荐值 |
|------|------|------|--------|
| timestep | `<option>` | 仿真时间步长(秒) | 0.001-0.005 |
| iterations | `<option>` | 求解器迭代次数 | 20-100 |
| tolerance | `<option>` | 求解器容差 | 1e-10 |
| friction | `<geom>` | 摩擦系数 [滑动, 转动, 滚动] | [0.7, 0.005, 0.0001] |
| damping | `<joint>` | 关节阻尼 | 0.5-2.0 |
| armature | `<joint>` | 关节转动惯量 | 0.01-0.15 |
| solimp | `<geom>` | 接触阻抗参数 | [0.9, 0.95, 0.001] |
| solref | `<geom>` | 接触参考参数 | [0.02, 1] |

## 许可证
本项目仅供学习和研究使用。

## 更新日志
- **v1.0** (2026-01-15): 初始版本发布
  - 完整的卧室场景
  - 6自由度机械臂
  - 移动小车
  - 基础家具和装饰

## 联系与支持
如有问题或建议，请参考 MuJoCo 官方文档：https://mujoco.readthedocs.io/
