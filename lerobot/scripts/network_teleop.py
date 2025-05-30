from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
import argparse
import tqdm
import socket
import struct
import time
import numpy as np

def _recv_exact(sock: socket.socket, n: int) -> bytes:
    """Receive exactly *n* bytes or raise RuntimeError if the peer closes."""
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise RuntimeError("Socket closed while receiving data")
        data += chunk
    return data


def spin(robot, seconds: float, frequency: float, mode: str, conn: socket.socket | None):
    """
    Run tele‑operation for *seconds* seconds at *frequency* Hz.

    - mode == "leader": read positions from leader arm and stream to *conn*.
    - mode == "follower": receive positions from *conn* and write to follower arm.
    - mode == "None"   : local: copy leader → follower in the same process.
    """
    period = 1.0 / frequency
    end_time = time.perf_counter() + seconds
    with tqdm.tqdm(total=seconds, unit="s") as pbar:
        last_update = time.perf_counter()
        while time.perf_counter() < end_time:
            loop_start = time.perf_counter()

            if mode == "leader":
                leader_pos = robot.leader_arms["main"].read("Present_Position").astype(np.float32)
                payload = leader_pos.tobytes()
                conn.sendall(struct.pack('I', len(leader_pos)) + payload)

            elif mode == "follower":
                # first 4 bytes = uint32 length
                n = struct.unpack('I', _recv_exact(conn, 4))[0]
                data = _recv_exact(conn, n * 4)
                goal = np.frombuffer(data, dtype=np.float32)
                robot.follower_arms["main"].write("Goal_Position", goal)

            else:  # mode == "None"
                leader_pos = robot.leader_arms["main"].read("Present_Position")
                robot.follower_arms["main"].write("Goal_Position", leader_pos)

            # update progress bar
            now = time.perf_counter()
            pbar.update(now - last_update)
            last_update = now

            # maintain loop period
            sleep_t = period - (time.perf_counter() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50007, required=True, help="Set socket configs")
    parser.add_argument("--network_mode", type=str, default="None", help="Set teleop network mode: leader, follower, or None")
    args = parser.parse_args()

    robot_config = So100RobotConfig()

    robot = ManipulatorRobot(
        robot_config,
    )
    robot.connect()  # establish connection before teleop

    mode = args.network_mode.lower()
    conn = None
    if mode == "leader":
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", args.port))
        srv.listen(1)
        print(f"[leader] waiting for follower on port {args.port} …")
        conn, _ = srv.accept()
        print("[leader] follower connected")
    elif mode == "follower":
        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"[follower] connecting to leader on port {args.port} …")
        conn.connect(("localhost", args.port))
        print("[follower] connected")

    try:
        spin(robot, 10, 100, mode, conn)
    finally:
        if conn is not None:
            conn.close()