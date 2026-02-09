from isaacsim import SimulationApp
import os
import hydra
import rclpy
import torch
import time
import math

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")

@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    # launch omniverse app
    simulation_app = SimulationApp({
        "headless": False,
        "anti_aliasing": cfg.sim_app.anti_aliasing,
        "width": cfg.sim_app.width,
        "height": cfg.sim_app.height,
        "hide_ui": cfg.sim_app.hide_ui,
    })

    import omni
    import carb
    import go2.go2_ctrl as go2_ctrl
    import ros2.go2_ros2_bridge as go2_ros2_bridge
    from go2.go2_env import Go2RSLEnvCfg, camera_follow
    import env.sim_env as sim_env
    import go2.go2_sensors as go2_sensors

    # Go2 Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    # ── Patch: ensure .device is a torch.device, not a string ────────────────
    if hasattr(go2_env_cfg, "device") and isinstance(go2_env_cfg.device, str):
        go2_env_cfg.device = torch.device(go2_env_cfg.device)
    # ───────────────────────────────────────────────────────────────────────────

    go2_env_cfg.scene.num_envs = cfg.num_envs
    go2_env_cfg.decimation = math.ceil(1.0 / go2_env_cfg.sim.dt / cfg.freq)
    go2_env_cfg.sim.render_interval = go2_env_cfg.decimation

    go2_ctrl.init_base_vel_cmd(cfg.num_envs)
    # pick your policy
    # env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)

    # Simulation environment
    if cfg.env_name == "obstacle-dense":
        sim_env.create_obstacle_dense_env()
    elif cfg.env_name == "obstacle-medium":
        sim_env.create_obstacle_medium_env()
    elif cfg.env_name == "obstacle-sparse":
        sim_env.create_obstacle_sparse_env()
    elif cfg.env_name == "warehouse":
        sim_env.create_warehouse_env()
    elif cfg.env_name == "warehouse-forklifts":
        sim_env.create_warehouse_forklifts_env()
    elif cfg.env_name == "warehouse-shelves":
        sim_env.create_warehouse_shelves_env()
    elif cfg.env_name == "full-warehouse":
        sim_env.create_full_warehouse_env()
    elif cfg.env_name == "farm":
        sim_env.create_farm_env()

    # Sensor setup
    sm = go2_sensors.SensorManager(cfg.num_envs)
    lidar_annotators = sm.add_rtx_lidar()
    cameras = sm.add_camera(cfg.freq)
    imu_sensors = sm.add_imu(cfg.freq)  


    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(),
        go2_ctrl.sub_keyboard_event,
    )

    # ROS2 Bridge
    rclpy.init()
    dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, imu_sensors, cfg)

    # Run simulation loop
    sim_step_dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    obs, _ = env.reset()
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

            # ROS2 data
            dm.pub_ros2_data()
            rclpy.spin_once(dm)

            # Camera follow
            if cfg.camera_follow:
                camera_follow(env)

            # limit loop time
            elapsed = time.time() - start_time
            if elapsed < sim_step_dt:
                time.sleep(sim_step_dt - elapsed)

        actual_loop_time = time.time() - start_time
        rtf = min(1.0, sim_step_dt / actual_loop_time)
        print(f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}", end="", flush=True)

    dm.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()
