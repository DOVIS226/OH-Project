#!/usr/bin/env python3
"""
MuJoCo 卧室场景测试脚本

这个脚本演示如何加载和运行卧室场景，并控制机械臂进行简单运动。

使用方法:
    python test_scene.py [options]

选项:
    --no-demo           跳过机械臂演示动画
    --fps FPS           设置目标帧率 (默认: 60)
    --duration SEC      演示持续时间，单位秒 (默认: 50)
    --camera NAME       初始相机视角 (默认: fixed_overview)
    --verbose           显示详细调试信息
    --info              仅打印场景信息后退出

依赖:
    pip install mujoco
"""

import os
import sys
import argparse
import logging
import numpy as np
import mujoco
import mujoco.viewer
import time
from pathlib import Path

# 配置日志系统
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)

logger = logging.getLogger(__name__)


class SimulationConfig:
    """仿真配置类"""

    def __init__(self, args):
        self.enable_demo = not args.no_demo
        self.target_fps = args.fps
        self.demo_duration = args.duration
        self.initial_camera = args.camera
        self.verbose = args.verbose
        self.info_only = args.info

        # 相机预设位置
        self.camera_presets = {
            'fixed_overview': {'azimuth': 135, 'elevation': -20, 'distance': 12, 'lookat': [0, 0, 1]},
            'bedroom_view': {'azimuth': 45, 'elevation': -15, 'distance': 6, 'lookat': [-3, -2, 1]},
            'living_room_view': {'azimuth': 180, 'elevation': -25, 'distance': 8, 'lookat': [0, 2, 1]},
            'kitchen_view': {'azimuth': 225, 'elevation': -20, 'distance': 7, 'lookat': [9, -8, 1]},
            'robot_view': {'azimuth': 45, 'elevation': -10, 'distance': 2, 'lookat': [3, -2, 1]},
        }


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='MuJoCo 卧室场景仿真',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  %(prog)s                          # 使用默认设置运行
  %(prog)s --no-demo                # 跳过演示动画
  %(prog)s --fps 30                 # 以30FPS运行
  %(prog)s --camera robot_view      # 从机器人视角启动
  %(prog)s --info                   # 仅显示场景信息
  %(prog)s --verbose                # 显示详细日志
        """
    )

    parser.add_argument('--no-demo', action='store_true',
                        help='跳过机械臂演示动画')
    parser.add_argument('--fps', type=int, default=60,
                        help='目标帧率 (默认: 60)')
    parser.add_argument('--duration', type=float, default=50.0,
                        help='演示持续时间（秒）(默认: 50)')
    parser.add_argument('--camera', type=str, default='fixed_overview',
                        choices=['fixed_overview', 'bedroom_view', 'living_room_view',
                                'kitchen_view', 'robot_view'],
                        help='初始相机视角 (默认: fixed_overview)')
    parser.add_argument('--verbose', action='store_true',
                        help='显示详细调试信息')
    parser.add_argument('--info', action='store_true',
                        help='仅打印场景信息后退出')
    parser.add_argument('--scene', type=str, default='bedroom_scene.xml',
                        help='场景文件名 (默认: bedroom_scene.xml)')

    return parser.parse_args()


def load_model(xml_path: Path):
    """加载 MuJoCo 模型

    Args:
        xml_path: XML 场景文件路径

    Returns:
        tuple: (model, data) 元组

    Raises:
        FileNotFoundError: 场景文件不存在
        Exception: 模型加载失败
    """
    if not xml_path.exists():
        raise FileNotFoundError(f"场景文件不存在: {xml_path}")

    logger.info(f"正在加载场景: {xml_path}")

    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        logger.info("场景加载成功!")
        return model, data
    except Exception as e:
        logger.error(f"模型加载失败: {e}")
        raise


def print_model_info(model):
    """打印模型详细信息

    Args:
        model: MuJoCo 模型对象
    """
    logger.info("=" * 60)
    logger.info("场景统计信息:")
    logger.info(f"  物体数量: {model.nbody}")
    logger.info(f"  关节数量: {model.njnt}")
    logger.info(f"  执行器数量: {model.nu}")
    logger.info(f"  传感器数量: {model.nsensor}")
    logger.info(f"  几何体数量: {model.ngeom}")
    logger.info(f"  相机数量: {model.ncam}")
    logger.info("=" * 60)


def print_actuators(model):
    """打印执行器列表

    Args:
        model: MuJoCo 模型对象
    """
    logger.info("\n执行器列表:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        logger.info(f"  [{i:2d}] {actuator_name}")


def print_detailed_info(model):
    """打印详细场景信息

    Args:
        model: MuJoCo 模型对象
    """
    logger.info("\n" + "=" * 60)
    logger.info("详细场景信息")
    logger.info("=" * 60)

    # 物体列表
    logger.info("\n物体列表:")
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name:
            logger.info(f"  {i:3d}: {body_name}")

    # 关节列表
    logger.info("\n关节列表:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]
        type_names = {0: 'free', 1: 'ball', 2: 'slide', 3: 'hinge'}
        type_str = type_names.get(joint_type, 'unknown')
        logger.info(f"  {i:3d}: {joint_name:<25s} (类型: {type_str})")

    # 传感器列表
    if model.nsensor > 0:
        logger.info("\n传感器列表:")
        for i in range(model.nsensor):
            sensor_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            if sensor_name:
                logger.info(f"  {i:3d}: {sensor_name}")

    # 相机列表
    if model.ncam > 0:
        logger.info("\n相机列表:")
        for i in range(model.ncam):
            camera_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            if camera_name:
                logger.info(f"  {i:3d}: {camera_name}")


def compute_robot_control(step: int, config: SimulationConfig) -> np.ndarray:
    """计算机械臂控制信号

    Args:
        step: 当前仿真步数
        config: 仿真配置

    Returns:
        控制信号数组 (8个元素: 6个关节 + 2个夹爪)
    """
    ctrl = np.zeros(8)

    if not config.enable_demo:
        return ctrl

    t = step * 0.01  # 时间

    # 控制各个关节 - 使用更平滑的正弦波组合
    ctrl[0] = 0.5 * np.sin(t * 0.5)      # 基座旋转
    ctrl[1] = 0.3 * np.sin(t * 0.7)      # 肩关节
    ctrl[2] = 0.4 * np.sin(t * 0.9)      # 肘关节
    ctrl[3] = 0.2 * np.sin(t * 1.2)      # 腕关节1
    ctrl[4] = 0.2 * np.sin(t * 1.5)      # 腕关节2
    ctrl[5] = 0.3 * np.sin(t * 2.0)      # 腕关节3

    # 夹爪开合演示
    gripper_state = 0.3 * (np.sin(t * 0.3) + 1) / 2
    ctrl[6] = gripper_state              # 左夹爪
    ctrl[7] = gripper_state              # 右夹爪

    return ctrl


def setup_camera(viewer, config: SimulationConfig):
    """设置相机视角

    Args:
        viewer: MuJoCo viewer 对象
        config: 仿真配置
    """
    if config.initial_camera in config.camera_presets:
        preset = config.camera_presets[config.initial_camera]
        viewer.cam.azimuth = preset['azimuth']
        viewer.cam.elevation = preset['elevation']
        viewer.cam.distance = preset['distance']
        viewer.cam.lookat[:] = preset['lookat']
        logger.info(f"相机设置为: {config.initial_camera}")


def main():
    """主函数：加载场景并运行仿真"""

    # 解析命令行参数
    args = parse_arguments()
    config = SimulationConfig(args)

    # 设置日志级别
    if config.verbose:
        logger.setLevel(logging.DEBUG)
        logger.debug("详细日志模式已启用")

    # 获取场景文件路径
    script_dir = Path(__file__).parent
    xml_path = script_dir / args.scene

    logger.info("=" * 60)
    logger.info("MuJoCo 卧室场景仿真")
    logger.info("=" * 60)

    try:
        # 加载模型
        model, data = load_model(xml_path)

        # 打印场景信息
        print_model_info(model)
        print_actuators(model)

        # 如果只显示信息，则退出
        if config.info_only:
            print_detailed_info(model)
            logger.info("信息模式：程序退出")
            return 0

        # 显示使用提示
        logger.info("\n启动可视化窗口...")
        logger.info("提示:")
        logger.info("  - 按住鼠标左键拖动可旋转视角")
        logger.info("  - 按住鼠标右键拖动可平移视角")
        logger.info("  - 滚动鼠标滚轮可缩放")
        logger.info("  - 按 ESC 或关闭窗口退出")

        if config.enable_demo:
            logger.info(f"\n机械臂将执行 {config.demo_duration} 秒的摆动演示...")
        else:
            logger.info("\n演示模式已禁用")

        logger.info("")

        # 计算演示最大步数
        max_demo_steps = int(config.demo_duration / model.opt.timestep)

        # 性能统计
        fps_samples = []
        last_log_time = time.time()

        # 运行仿真
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # 设置相机
            setup_camera(viewer, config)

            step = 0

            while viewer.is_running():
                step_start = time.time()

                # 计算控制命令
                if step < max_demo_steps:
                    data.ctrl[:] = compute_robot_control(step, config)

                    # 每2秒打印一次状态
                    if time.time() - last_log_time >= 2.0:
                        avg_fps = np.mean(fps_samples[-60:]) if fps_samples else 0
                        logger.info(f"步数: {step:6d} | 时间: {data.time:6.2f}s | "
                                  f"基座角度: {np.rad2deg(data.qpos[0]):6.1f}° | "
                                  f"FPS: {avg_fps:.1f}")
                        last_log_time = time.time()

                elif step == max_demo_steps and config.enable_demo:
                    logger.info("\n✓ 演示结束！现在您可以手动控制或关闭窗口。")
                    data.ctrl[:] = 0

                # 执行物理仿真步
                mujoco.mj_step(model, data)

                # 同步渲染
                viewer.sync()

                step += 1

                # 计算实际FPS
                frame_time = time.time() - step_start
                if frame_time > 0:
                    fps_samples.append(1.0 / frame_time)
                    if len(fps_samples) > 120:
                        fps_samples.pop(0)

                # 控制帧率
                target_frame_time = 1.0 / config.target_fps
                time_until_next_step = target_frame_time - frame_time
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        logger.info("\n✓ 仿真已结束")
        return 0

    except FileNotFoundError as e:
        logger.error(str(e))
        return 1
    except KeyboardInterrupt:
        logger.info("\n用户中断仿真")
        return 0
    except Exception as e:
        logger.error(f"\n发生错误: {e}")
        if config.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
