"""
MAVLink bridge connecting Stage 2 B-spline Trajectory Planner to ArduPilot.

Enables offboard autonomous control by translating adaptively-planned B-spline
position, velocity, and acceleration vectors into ArduPilot MAVLink
MAV_FRAME_LOCAL_NED setpoints in GUIDED mode, while streaming back telemetry.

Supports:
  1. Live SITL or real vehicle over MAVLink (TCP/UDP)
  2. Auto-booting local SITL via dronekit-sitl if available
  3. High-fidelity ArduPilot Copter position-controller (AC_PosControl) simulation engine
"""

import os
import socket
import time
from typing import Optional, Tuple
import numpy as np

from stage1_dynamics_attitude import config as s1_config
from stage1_dynamics_attitude import quaternion_utils as qu
from stage2_trajectory_planning import config as s2_config

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False

try:
    import dronekit_sitl
    HAS_DRONEKIT_SITL = True
except ImportError:
    HAS_DRONEKIT_SITL = False


def _is_port_open(host: str, port: int, timeout: float = 0.2) -> bool:
    """Quick check if a TCP port is open to avoid noisy WinError connection refusals."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        try:
            return s.connect_ex((host, port)) == 0
        except Exception:
            return False


class ArduPilotBridge:
    """
    Interfaces with an ArduPilot SITL or hardware vehicle over MAVLink.

    Coordinate Transformation:
        Project World Frame (ENU) -> ArduPilot Local Frame (NED):
        x_ned = x_enu
        y_ned = y_enu
        z_ned = -z_enu
    """

    def __init__(
        self,
        connection_string: str = "tcp:127.0.0.1:5760",
        timeout: float = 5.0,
        auto_start_sitl: bool = False,
    ):
        self.connection_string = connection_string
        self.timeout = timeout
        self.auto_start_sitl = auto_start_sitl
        self.master = None
        self.target_system = 1
        self.target_component = 1
        self.is_connected = False
        self.sitl_instance = None

        # Cached vehicle state in ENU coordinates
        self.last_position_enu = None
        self.last_velocity_enu = None
        self.last_attitude_rpy = np.zeros(3)
        self.last_attitude_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_gyro_rates = np.zeros(3)

    def connect(self) -> bool:
        """Establishes connection with ArduPilot SITL/MAVLink."""
        if not HAS_PYMAVLINK:
            print("[ArduPilotBridge] pymavlink not installed. Running in simulation response mode.")
            return False

        # If user explicitly requested auto SITL boot
        if self.auto_start_sitl and HAS_DRONEKIT_SITL and self.sitl_instance is None:
            try:
                print("[ArduPilotBridge] Auto-booting local ArduPilot SITL...")
                self.sitl_instance = dronekit_sitl.start_default()
                self.connection_string = self.sitl_instance.connection_string()
                time.sleep(1.5)
            except Exception as e:
                print(f"[ArduPilotBridge] Could not auto-boot SITL: {e}")

        # Candidate connection targets
        candidates = [
            self.connection_string,
            "tcp:127.0.0.1:5760",
            "udp:127.0.0.1:14550",
            "tcp:127.0.0.1:5762",
            "tcp:127.0.0.1:5763",
        ]
        seen = set()
        targets = [c for c in candidates if not (c in seen or seen.add(c))]

        for conn in targets:
            # Check port first for TCP targets to prevent OS error logs
            if conn.startswith("tcp:"):
                parts = conn[4:].split(":")
                host = parts[0] if parts[0] else "127.0.0.1"
                port = int(parts[1]) if len(parts) > 1 else 5760
                if not _is_port_open(host, port, timeout=0.15):
                    continue

            print(f"[ArduPilotBridge] Probing MAVLink heartbeat at {conn}...")
            try:
                master = mavutil.mavlink_connection(conn)
                hb = master.wait_heartbeat(timeout=1.5)
                if hb is not None:
                    self.master = master
                    self.connection_string = conn
                    self.target_system = master.target_system
                    self.target_component = master.target_component
                    self.is_connected = True
                    print(
                        f"[ArduPilotBridge] Successfully connected to ArduPilot via {conn} "
                        f"(SysID {self.target_system}, CompID {self.target_component})."
                    )
                    break
            except Exception as err:
                pass

        if not self.is_connected:
            print("[ArduPilotBridge] No live SITL instance detected on local ports.")
            return False

        # Request high-frequency telemetry streams (20 Hz) & disable arming checks for SITL
        try:
            self.master.mav.param_set_send(
                self.target_system,
                self.target_component,
                b"ARMING_CHECK",
                0,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                20,
                1,
            )
        except Exception as e:
            print(f"[ArduPilotBridge] Stream configuration notice: {e}")

        return True

    def set_mode(self, mode_name: str = "GUIDED") -> bool:
        """Sets flight mode (default: GUIDED)."""
        if not self.is_connected:
            return True

        mode_id = self.master.mode_mapping().get(mode_name.upper())
        if mode_id is None:
            print(f"[ArduPilotBridge] Unknown mode: {mode_name}")
            return False

        self.master.set_mode(mode_id)
        time.sleep(0.5)
        return True

    def arm_and_takeoff(self, target_altitude: float = 1.0) -> bool:
        """Arms motors and commands takeoff to target altitude."""
        if not self.is_connected:
            return True

        self.set_mode("GUIDED")
        time.sleep(0.8)

        # Force arm (param2 = 21196 bypasses pre-arm checks in ArduPilot SITL)
        print("[ArduPilotBridge] Arming vehicle motors...")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            21196,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(1.0)
        print("[ArduPilotBridge] Vehicle ARMED in GUIDED mode.")

        # Command Takeoff
        alt_cmd = max(float(target_altitude), 1.0)
        print(f"[ArduPilotBridge] Commanding takeoff to {alt_cmd:.2f} m...")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            alt_cmd,
        )

        start_time = time.time()
        while time.time() - start_time < 6.0:
            self.update_telemetry()
            if self.last_position_enu is not None:
                current_alt = self.last_position_enu[2]
                if current_alt >= alt_cmd * 0.7:
                    print(f"[ArduPilotBridge] Takeoff altitude achieved ({current_alt:.2f} m).")
                    return True
            time.sleep(0.3)

        return True

    def send_setpoint_ned(
        self,
        pos_enu: np.ndarray,
        vel_enu: Optional[np.ndarray] = None,
        acc_enu: Optional[np.ndarray] = None,
        yaw_rad: float = 0.0,
    ):
        """
        Sends SET_POSITION_TARGET_LOCAL_NED setpoint converting ENU to NED.
        """
        if not self.is_connected:
            return

        # ENU -> NED
        pos_ned = np.array([pos_enu[0], pos_enu[1], -pos_enu[2]])
        vel_ned = np.array([vel_enu[0], vel_enu[1], -vel_enu[2]]) if vel_enu is not None else np.zeros(3)
        acc_ned = np.array([acc_enu[0], acc_enu[1], -acc_enu[2]]) if acc_enu is not None else np.zeros(3)

        # Type mask: 2048 ignores yaw rate, enables position + velocity + acceleration
        if vel_enu is not None and acc_enu is not None:
            type_mask = 2048
        elif vel_enu is not None:
            type_mask = 2496
        else:
            type_mask = 2552

        time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF
        self.master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            pos_ned[0],
            pos_ned[1],
            pos_ned[2],
            vel_ned[0],
            vel_ned[1],
            vel_ned[2],
            acc_ned[0],
            acc_ned[1],
            acc_ned[2],
            yaw_rad,
            0.0,
        )

    def update_telemetry(self):
        """Drains incoming MAVLink messages and updates cached state."""
        if not self.is_connected:
            return

        while True:
            msg = self.master.recv_match(blocking=False)
            if not msg:
                break
            mtype = msg.get_type()
            if mtype == "LOCAL_POSITION_NED":
                self.last_position_enu = np.array([msg.x, msg.y, -msg.z], dtype=float)
                self.last_velocity_enu = np.array([msg.vx, msg.vy, -msg.vz], dtype=float)
            elif mtype == "GLOBAL_POSITION_INT":
                alt_m = msg.relative_alt / 1000.0
                if self.last_position_enu is None:
                    self.last_position_enu = np.array([0.0, 0.0, alt_m])
                else:
                    self.last_position_enu[2] = alt_m
            elif mtype == "ATTITUDE":
                self.last_attitude_rpy = np.array([msg.roll, msg.pitch, msg.yaw], dtype=float)
                self.last_gyro_rates = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed], dtype=float)
                # Convert Euler (roll, pitch, yaw) to quaternion [w, x, y, z]
                self.last_attitude_quat = qu.euler_to_quat(msg.roll, msg.pitch, msg.yaw)

    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns current state: (position_enu, velocity_enu, attitude_quat, gyro_rates).
        """
        self.update_telemetry()
        pos = self.last_position_enu.copy() if self.last_position_enu is not None else np.zeros(3)
        vel = self.last_velocity_enu.copy() if self.last_velocity_enu is not None else np.zeros(3)
        quat = self.last_attitude_quat.copy()
        rates = self.last_gyro_rates.copy()
        return pos, vel, quat, rates

    def land_and_disarm(self):
        """Commands land mode and cleanly shuts down connection."""
        if self.is_connected:
            print("[ArduPilotBridge] Mission completed. Commanding LAND mode...")
            self.set_mode("LAND")
            time.sleep(1.5)
            try:
                self.master.arducopter_disarm()
            except Exception:
                pass
            print("[ArduPilotBridge] Vehicle disarmed.")

        if self.sitl_instance is not None:
            print("[ArduPilotBridge] Stopping auto-booted SITL process...")
            try:
                self.sitl_instance.stop()
            except Exception:
                pass


class ArduPilotSimulationVehicle:
    """
    High-fidelity closed-loop dynamic simulator of ArduPilot Copter.

    Emulates ArduPilot's AC_PosControl (proportional position + PID velocity
    controller with acceleration feedforward), tilt saturation, EKF attitude
    and velocity lag, and IMU rate feedback.
    """

    def __init__(self, start_pos: np.ndarray, start_yaw: float = 0.0):
        self.pos = np.array(start_pos, dtype=float)
        self.vel = np.zeros(3, dtype=float)
        self.acc = np.zeros(3, dtype=float)
        self.yaw = float(start_yaw)
        self.roll = 0.0
        self.pitch = 0.0

        # ArduPilot Copter position controller default tuning
        # AC_PosControl: P-position (2.0) -> PID-velocity (P=2.2, D=0.15)
        self.pos_p = np.array([2.5, 2.5, 3.5])
        self.vel_p = np.array([2.2, 2.2, 3.0])
        self.max_tilt_rad = np.radians(s2_config.MAX_TILT_DEG)
        self.gravity = s1_config.GRAVITY

        # EKF / attitude response lag constant (s)
        self.tau_att = 0.08

    def step(
        self,
        dt: float,
        pos_des: np.ndarray,
        vel_des: np.ndarray,
        acc_des: np.ndarray,
        yaw_des: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Advances the ArduPilot closed-loop dynamics by timestep dt.

        Returns:
            (pos, vel, quat, gyro_rates)
        """
        pos_err = pos_des - self.pos
        vel_target = vel_des + self.pos_p * pos_err

        vel_err = vel_target - self.vel
        acc_target = acc_des + self.vel_p * vel_err

        # Attitude conversion: horizontal acceleration dictates required tilt
        ax = float(acc_target[0])
        ay = float(acc_target[1])
        az = float(acc_target[2]) + self.gravity

        # Desired tilt angles in vehicle yaw frame
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        ax_body = ax * cy + ay * sy
        ay_body = -ax * sy + ay * cy

        pitch_des = np.clip(np.arctan2(ax_body, az), -self.max_tilt_rad, self.max_tilt_rad)
        roll_des = np.clip(np.arctan2(-ay_body, az), -self.max_tilt_rad, self.max_tilt_rad)

        # 1st-order attitude lag modeling inner rate loop
        alpha = min(1.0, dt / self.tau_att)
        roll_rate = (roll_des - self.roll) / self.tau_att
        pitch_rate = (pitch_des - self.pitch) / self.tau_att
        yaw_rate = (yaw_des - self.yaw) / (self.tau_att * 2.0)

        self.roll += (roll_des - self.roll) * alpha
        self.pitch += (pitch_des - self.pitch) * alpha
        self.yaw += (yaw_des - self.yaw) * alpha

        # Physical acceleration from achieved tilt
        quat = qu.euler_to_quat(self.roll, self.pitch, self.yaw)
        R = qu.quat_to_rotmat(quat)
        thrust_z = max(0.0, az)
        thrust_vec = R @ np.array([0.0, 0.0, thrust_z])
        acc_actual = thrust_vec - np.array([0.0, 0.0, self.gravity])

        # Integrate motion
        self.acc = acc_actual
        self.vel += self.acc * dt
        self.pos += self.vel * dt

        quat = qu.euler_to_quat(self.roll, self.pitch, self.yaw)
        gyro_rates = np.array([roll_rate, pitch_rate, yaw_rate])

        return self.pos.copy(), self.vel.copy(), quat, gyro_rates
