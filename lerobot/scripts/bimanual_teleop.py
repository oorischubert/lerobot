from lerobot.common.robot_devices.robots.configs import So100BimanualRobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
import argparse
import tqdm

def spin(robot,seconds,frequency):
    for _ in tqdm.tqdm(range(seconds * frequency)):
        robot.teleop_step()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50007, required=True, help="Set socket configs")
    parser.add_argument("--network_mode", type=str, default="None", help="Set teleop network mode: leader, follower, or None")
    args = parser.parse_args()

    robot_config = So100BimanualRobotConfig()

    robot = ManipulatorRobot(
        robot_config,
        teleop_network_mode=args.network_mode,
        teleop_host="localhost",
        teleop_port=args.port
    )
    
    spin(robot,10,10)
    