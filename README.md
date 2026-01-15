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
pip install mujoco-python-viewer  # 可选，用于可视化
```

### 加载场景
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
```python
# 设置关节控制命令
data.ctrl[0] = 1.0  # 控制基座旋转
data.ctrl[1] = 0.5  # 控制肩关节
# ... 设置其他关节

# 控制夹爪
data.ctrl[6] = 0.3  # 左夹爪
data.ctrl[7] = 0.3  # 右夹爪

# 执行仿真步
mujoco.mj_step(model, data)
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
1. 如果不需要移动小车，可以注释掉对应的 `<body>` 块以提高性能
2. 降低 `timestep` 值可提高精度，但会降低仿真速度
3. 调整 `<visual>` 中的 `quality` 参数平衡渲染质量与性能

## 已知问题与限制
1. 场景中的家具为静态物体，不可移动
2. 移动小车使用自由关节（freejoint），需要额外的控制器实现移动
3. 纹理使用内置生成器，如需更真实效果请使用外部纹理图片

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
