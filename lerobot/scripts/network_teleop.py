from lerobot.common.robot_devices.robots.configs import So100RobotConfig, So100BimanualRobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
import argparse
import tqdm
import socket
import time
import numpy as np
import sys

LEADER_ADDR = "100.87.198.21" # telepi
UPDATE_FREQ = 50

def _recv_exact(sock: socket.socket, n: int) -> bytes:
    """Receive exactly *n* bytes or raise RuntimeError if the peer closes."""
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise RuntimeError("Socket closed while receiving data")
        data += chunk
    return data

def _motor_count(bus):
    if hasattr(bus, "num_motors"):
        return bus.num_motors
    if hasattr(bus, "motors"):
        return len(bus.motors)
    if hasattr(bus, "motor_ids"):
        return len(bus.motor_ids)
    raise AttributeError("Cannot determine motor count for this bus")

def spin(robot, seconds: float | None, frequency: float, mode: str, conn: socket.socket | None):
    """
    Tele-op loop for `seconds` at `frequency` Hz.

    * leader   - sends all arms found in `robot.leader_arms`
    * follower - receives data for all arms in `robot.follower_arms`
    * None     - local mirroring for arm names present on both sides
    """
    if mode == "leader":
        arm_keys = list(robot.leader_arms.keys())
    elif mode == "follower":
        arm_keys = list(robot.follower_arms.keys())
    else:                               # local
        arm_keys = [k for k in robot.leader_arms if k in robot.follower_arms]

    if not arm_keys:
        raise RuntimeError("No arms available for the selected mode")

    # how many motors per arm (needed for packing / unpacking)
    dims = {k: (_motor_count(robot.leader_arms[k]) if mode != "follower"
            else _motor_count(robot.follower_arms[k]))
        for k in arm_keys}

    # byte-level header: [n_arms:uint8] + repeating
    #   [key_len:uint8][key_bytes][n_vals:uint8]
    header_bytes = bytearray()
    header_bytes.append(len(arm_keys))
    for k in arm_keys:
        key_b = k.encode()
        header_bytes.append(len(key_b))
        header_bytes.extend(key_b)
        header_bytes.append(dims[k])

    period = 1.0 / frequency
    start_t  = time.perf_counter()
    end_t = start_t + seconds if seconds is not None else float("inf")
    with tqdm.tqdm(
        total=seconds or float("inf"),
        unit="s",
        bar_format="{l_bar}{bar}| {n:.1f}/{total:.0f}{unit} " if seconds is not None else "{l_bar}{bar}| ∞ "
    ) as bar:
        prev = time.perf_counter()
        while time.perf_counter() < end_t:
            t0 = time.perf_counter()

            if mode == "leader":
                assert conn is not None
                # concatenate all leader positions in the agreed order
                blob = np.concatenate(
                    [robot.leader_arms[k].read("Present_Position").astype(np.float32)
                     for k in arm_keys]
                ).tobytes()
                conn.sendall(header_bytes + blob)

            elif mode == "follower":
                assert conn is not None
                # read header only once per loop (it is fixed-size)
                hdr = _recv_exact(conn, len(header_bytes))
                data_len = sum(dims.values()) * 4  # float32 = 4 bytes
                buf = _recv_exact(conn, data_len)
                flat = np.frombuffer(buf, dtype=np.float32)

                # slice and dispatch to each follower arm
                idx = 0
                for k in arm_keys:
                    n = dims[k]
                    robot.follower_arms[k].write("Goal_Position", flat[idx:idx+n])
                    idx += n

            else:  # local mirror
                for k in arm_keys:
                    robot.follower_arms[k].write(
                        "Goal_Position",
                        robot.leader_arms[k].read("Present_Position"),
                    )

            # progress-bar housekeeping & rate control
            now = time.perf_counter()
            if seconds is not None:
                 bar.update(now - prev)
            prev = now
            time.sleep(max(0.0, period - (now - t0)))

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50007, required=True, help="Set socket configs")
    parser.add_argument("--mode",type=str, default="None", help="Set teleop network mode: leader, follower, or None")
    parser.add_argument("--robot",type=str, default="so100", help="Robot type: [so100, so100_bimanual]")
    parser.add_argument(
    "-t", "--runtime", type=float, default=None,
    help="Duration in seconds. Omit for infinite run (stop with Ctrl-C).",
)
    args = parser.parse_args()
    if args.robot == "so100":
        robot_config = So100RobotConfig()
    elif args.robot == "so100_bimanual":
        robot_config = So100BimanualRobotConfig()
    else:
        print("Robot not integrated, select from: [so100, so100_bimanual]")
        sys.exit(1)
        
    mode = args.mode.lower()
    conn = None
    
    robot_config.cameras={} # temp!
    
    if mode == "leader":
        robot_config.follower_arms={} # empty follower arm list
        robot_config.cameras={} # empty camera list
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", args.port))
        srv.listen(1)
        print(f"[leader] waiting for follower/s on port {args.port} …")
        conn, _ = srv.accept()
        print("[leader] follower connected")
        assert conn is not None
    elif mode == "follower":
        robot_config.leader_arms={} # empty leader arm list
        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"[follower] connecting to leader/s on port {args.port} …")
        conn.connect((LEADER_ADDR, args.port))
        print("[follower] connected")
        assert conn is not None

    robot = ManipulatorRobot(
        robot_config,
    )
    robot.connect()  # establish connection before teleop

    try:
        spin(robot, args.runtime, UPDATE_FREQ, mode, conn)
    finally:
        if conn is not None:
            conn.close()

# Running network_teleop:  
# python -m lerobot.scripts.network_teleop --port <port> --robot <robot> --mode <mode> -t <s>