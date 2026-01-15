#!/usr/bin/env python3
"""
MuJoCo 卧室场景测试脚本

这个脚本演示如何加载和运行卧室场景，并控制机械臂进行简单运动。

使用方法:
    python test_scene.py

依赖:
    pip install mujoco
"""

import os
import numpy as np
import mujoco
import mujoco.viewer
import time


def main():
    """主函数：加载场景并运行仿真"""

    # 获取场景文件路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(script_dir, 'bedroom_scene.xml')

    # 检查文件是否存在
    if not os.path.exists(xml_path):
        print(f"错误：找不到场景文件 {xml_path}")
        return

    print("=" * 60)
    print("MuJoCo 卧室场景仿真")
    print("=" * 60)
    print(f"加载场景文件: {xml_path}")

    try:
        # 加载 MuJoCo 模型
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)

        print(f"✓ 场景加载成功！")
        print(f"  - 物体数量: {model.nbody}")
        print(f"  - 关节数量: {model.njnt}")
        print(f"  - 执行器数量: {model.nu}")
        print(f"  - 传感器数量: {model.nsensor}")
        print()

        # 打印关节信息
        print("机械臂关节列表:")
        for i in range(model.nu):
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            print(f"  [{i}] {actuator_name}")
        print()

        # 创建可视化窗口
        print("启动可视化窗口...")
        print("提示:")
        print("  - 按住鼠标左键拖动可旋转视角")
        print("  - 按住鼠标右键拖动可平移视角")
        print("  - 滚动鼠标滚轮可缩放")
        print("  - 按 ESC 或关闭窗口退出")
        print()
        print("机械臂将执行简单的摆动演示...")
        print()

        # 运行仿真
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # 设置相机初始位置
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            viewer.cam.distance = 8
            viewer.cam.lookat[:] = [0, 0, 1]

            step = 0
            demo_mode = True

            while viewer.is_running():
                step_start = time.time()

                if demo_mode and step < 5000:
                    # 演示模式：机械臂简单摆动
                    t = step * 0.01

                    # 控制各个关节
                    data.ctrl[0] = 0.5 * np.sin(t * 0.5)      # 基座旋转
                    data.ctrl[1] = 0.3 * np.sin(t * 0.7)      # 肩关节
                    data.ctrl[2] = 0.4 * np.sin(t * 0.9)      # 肘关节
                    data.ctrl[3] = 0.2 * np.sin(t * 1.2)      # 腕关节1
                    data.ctrl[4] = 0.2 * np.sin(t * 1.5)      # 腕关节2
                    data.ctrl[5] = 0.3 * np.sin(t * 2.0)      # 腕关节3

                    # 夹爪开合演示
                    gripper_state = 0.3 * (np.sin(t * 0.3) + 1) / 2
                    data.ctrl[6] = gripper_state              # 左夹爪
                    data.ctrl[7] = gripper_state              # 右夹爪

                    # 每500步打印一次信息
                    if step % 500 == 0:
                        print(f"仿真步数: {step}, 时间: {data.time:.2f}s, "
                              f"基座角度: {np.rad2deg(data.qpos[0]):.1f}°")

                elif demo_mode and step >= 5000:
                    print("\n演示结束！现在您可以手动控制或关闭窗口。")
                    demo_mode = False
                    # 重置控制命令
                    data.ctrl[:] = 0

                # 执行物理仿真步
                mujoco.mj_step(model, data)

                # 同步渲染
                viewer.sync()

                step += 1

                # 控制帧率（约60 FPS）
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        print("\n仿真已结束。")

    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


def print_scene_info():
    """打印场景详细信息（可选功能）"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(script_dir, 'bedroom_scene.xml')

    model = mujoco.MjModel.from_xml_path(xml_path)

    print("\n详细场景信息:")
    print("-" * 60)

    print("\n物体列表:")
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name:
            print(f"  {i}: {body_name}")

    print("\n关节列表:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]
        type_names = ['free', 'ball', 'slide', 'hinge']
        print(f"  {i}: {joint_name} (类型: {type_names[joint_type]})")

    print("\n几何体列表:")
    for i in range(model.ngeom):
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if geom_name:
            print(f"  {i}: {geom_name}")


if __name__ == "__main__":
    # 运行主程序
    main()

    # 如果需要查看详细信息，取消下面的注释
    # print_scene_info()
