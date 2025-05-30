from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
import argparse
import tqdm

def spin(robot,seconds,frequency):
    for _ in tqdm.tqdm(range(seconds * frequency)):
        #robot.teleop_step()
        leader_pos = robot.leader_arms["main"].read("Present_Position")
        robot.follower_arms["main"].write("Goal_Position", leader_pos)
        print(leader_pos)

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

    spin(robot,1000,10)