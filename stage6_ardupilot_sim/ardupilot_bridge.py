"""
MAVLink bridge connecting Stage 2 B-spline Trajectory Planner to ArduPilot SITL.

Enables full offboard control by translating generated B-spline position, velocity,
and acceleration vectors into ArduPilot MAVLink MAV_FRAME_LOCAL_NED setpoints.
"""

import time
import numpy as np

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False


class ArduPilotBridge:
    """
    Interfaces with an ArduPilot SITL vehicle over MAVLink (UDP/TCP).
    
    Coordinates transformation:
        Project World Frame (ENU) -> ArduPilot Local Frame (NED)
        x_ned = x_enu
        y_ned = y_enu
        z_ned = -z_enu
    """

    def __init__(self, connection_string: str = "udp:127.0.0.1:14550", timeout: float = 10.0):
        """
        Parameters
        ----------
        connection_string : str
            MAVLink target, e.g., 'udp:127.0.0.1:14550', 'tcp:127.0.0.1:5760', or 'com3'.
        timeout : float
            Connection timeout in seconds.
        """
        self.connection_string = connection_string
        self.timeout = timeout
        self.master = None
        self.target_system = 1
        self.target_component = 1
        self.is_connected = False

    def connect(self) -> bool:
        """Establishes connection with ArduPilot SITL and waits for heartbeat."""
        if not HAS_PYMAVLINK:
            print("[ArduPilotBridge] Warning: pymavlink is not installed. Running in mock mode.")
            return False

        print(f"[ArduPilotBridge] Connecting to SITL at {self.connection_string}...")
        try:
            self.master = mavutil.mavlink_connection(self.connection_string)
            hb = self.master.wait_heartbeat(timeout=self.timeout)
            if hb is None:
                print(f"[ArduPilotBridge] Failed to receive heartbeat within {self.timeout}s.")
                return False

            self.target_system = self.master.target_system
            self.target_component = self.master.target_component
            self.is_connected = True
            print(f"[ArduPilotBridge] Connected to ArduPilot (System {self.target_system}, Component {self.target_component}).")
            return True
        except Exception as e:
            print(f"[ArduPilotBridge] Connection error: {e}")
            return False

    def set_mode(self, mode_name: str = "GUIDED") -> bool:
        """Sets flight mode (default: GUIDED)."""
        if not self.is_connected:
            print(f"[ArduPilotBridge Mock] Set mode to {mode_name}")
            return True

        mode_id = self.master.mode_mapping().get(mode_name.upper())
        if mode_id is None:
            print(f"[ArduPilotBridge] Unknown mode: {mode_name}")
            return False

        self.master.set_mode(mode_id)
        print(f"[ArduPilotBridge] Commanded mode change to {mode_name}...")
        
        # Wait for mode change confirmation
        start_time = time.time()
        while time.time() - start_time < 5.0:
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                print(f"[ArduPilotBridge] Mode change acknowledged: {ack.result}")
                return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        return True

    def arm_and_takeoff(self, target_altitude: float = 1.0) -> bool:
        """Arms motors and commands takeoff to target altitude in NED coordinates."""
        if not self.is_connected:
            print(f"[ArduPilotBridge Mock] Armed and took off to altitude {target_altitude}m")
            return True

        self.set_mode("GUIDED")
        time.sleep(1.0)

        # Arm vehicle
        print("[ArduPilotBridge] Arming vehicle motors...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("[ArduPilotBridge] Vehicle ARMED.")

        # Send Takeoff command
        print(f"[ArduPilotBridge] Commanding takeoff to {target_altitude}m...")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_altitude
        )

        # Wait until target altitude is reached (NED z near -target_altitude)
        start_time = time.time()
        while time.time() - start_time < 15.0:
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
            if msg:
                current_alt = -msg.z
                print(f"[ArduPilotBridge] Current Altitude: {current_alt:.2f}m / Target: {target_altitude:.2f}m")
                if current_alt >= target_altitude * 0.95:
                    print("[ArduPilotBridge] Target altitude reached.")
                    return True
            time.sleep(0.2)

        return True

    def send_setpoint_ned(self, pos_enu: np.ndarray, vel_enu: np.ndarray = None, acc_enu: np.ndarray = None, yaw_rad: float = 0.0):
        """
        Sends SET_POSITION_TARGET_LOCAL_NED setpoint converting ENU to NED.

        Parameters
        ----------
        pos_enu : np.ndarray
            Desired position [x, y, z] in ENU frame (m).
        vel_enu : np.ndarray, optional
            Desired velocity [vx, vy, vz] in ENU frame (m/s).
        acc_enu : np.ndarray, optional
            Desired acceleration [ax, ay, az] in ENU frame (m/s^2).
        yaw_rad : float
            Desired yaw orientation in radians.
        """
        if not self.is_connected:
            return

        # ENU -> NED conversion
        pos_ned = np.array([pos_enu[0], pos_enu[1], -pos_enu[2]])
        vel_ned = np.array([vel_enu[0], vel_enu[1], -vel_enu[2]]) if vel_enu is not None else np.zeros(3)
        acc_ned = np.array([acc_enu[0], acc_enu[1], -acc_enu[2]]) if acc_enu is not None else np.zeros(3)

        # Type mask configuration:
        # Bit 0: Pos X, Bit 1: Pos Y, Bit 2: Pos Z
        # Bit 3: Vel X, Bit 4: Vel Y, Bit 5: Vel Z
        # Bit 6: Acc X, Bit 7: Acc Y, Bit 8: Acc Z
        # Bit 9: Use Force, Bit 10: Yaw, Bit 11: Yaw Rate
        # 0 = Use field, 1 = Ignore field
        if vel_enu is not None and acc_enu is not None:
            # Enable Pos + Vel + Acc (Ignore Yaw Rate: 2048) -> type_mask = 2048 (0x0800)
            type_mask = 2048
        elif vel_enu is not None:
            # Enable Pos + Vel (Ignore Acc: 64+128+256=448, Yaw Rate: 2048) -> type_mask = 2496
            type_mask = 2496
        else:
            # Enable Pos only (Ignore Vel: 56, Acc: 448, Yaw Rate: 2048) -> type_mask = 2552
            type_mask = 2552

        time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF

        self.master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            pos_ned[0], pos_ned[1], pos_ned[2],
            vel_ned[0], vel_ned[1], vel_ned[2],
            acc_ned[0], acc_ned[1], acc_ned[2],
            yaw_rad, 0.0
        )

    def get_telemetry(self) -> dict:
        """Fetches current position and attitude telemetry from ArduPilot."""
        if not self.is_connected:
            return None

        pos_msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        att_msg = self.master.recv_match(type='ATTITUDE', blocking=False)

        telemetry = {}
        if pos_msg:
            # NED -> ENU conversion
            telemetry['position_enu'] = np.array([pos_msg.x, pos_msg.y, -pos_msg.z])
            telemetry['velocity_enu'] = np.array([pos_msg.vx, pos_msg.vy, -pos_msg.vz])
        if att_msg:
            telemetry['attitude_rpy'] = np.array([att_msg.roll, att_msg.pitch, att_msg.yaw])

        return telemetry if telemetry else None

    def land_and_disarm(self):
        """Commands land mode and disarms vehicle."""
        if not self.is_connected:
            print("[ArduPilotBridge Mock] Landed and disarmed.")
            return

        print("[ArduPilotBridge] Commanding LAND mode...")
        self.set_mode("LAND")
        time.sleep(5.0)
        print("[ArduPilotBridge] Disarming motors...")
        self.master.arducopter_disarm()
        print("[ArduPilotBridge] Vehicle DISARMED.")
