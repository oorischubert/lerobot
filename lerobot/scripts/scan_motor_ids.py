# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Utility script to continuously scan for motor IDs on a motors bus.

Example usage:
```bash
python lerobot/scripts/scan_motor_ids.py --port /dev/ttyACM0 --brand feetech
```
"""

import argparse
import time
import sys

from lerobot.common.robot_devices.motors.configs import (
    DynamixelMotorsBusConfig,
    FeetechMotorsBusConfig,
)
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus


def get_motor_bus_cls(brand: str):
    if brand == "feetech":
        return FeetechMotorsBusConfig, FeetechMotorsBus
    elif brand == "dynamixel":
        return DynamixelMotorsBusConfig, DynamixelMotorsBus
    else:
        raise ValueError(f"Unsupported motor brand: {brand}")


def scan_ids(port: str, brand: str, model: str, baudrate: int, ids: list[int], delay: float = 0.5):
    config_cls, bus_cls = get_motor_bus_cls(brand)

    # create a dummy configuration so that the bus can perform reads
    dummy_id = ids[0] if ids else 1
    config = config_cls(port=port, motors={"scan": (dummy_id, model)})
    bus = bus_cls(config=config) # type: ignore

    bus.connect()
    if hasattr(bus, "set_bus_baudrate"):
        bus.set_bus_baudrate(baudrate)

    try:
        prev_len = 0  # track length of previous line for clean overwrite

        while True:
            present = bus.find_motor_indices(ids)
            if present:
                # Read present positions for the detected IDs
                positions = bus.read_with_motor_ids(bus.motor_models, present, "Present_Position")
                pairs = ", ".join(f"{idx}:{pos}" for idx, pos in zip(present, positions))
                line = f"ID→Position  {pairs}"
            else:
                line = "No motors detected"

            # overwrite the same terminal line
            sys.stdout.write("\r" + line.ljust(prev_len))
            sys.stdout.flush()
            prev_len = len(line)

            time.sleep(delay)
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect()


def parse_args():
    parser = argparse.ArgumentParser(description="Continuously scan for motor IDs")
    parser.add_argument("--port", type=str, required=True, help="Motors bus port")
    parser.add_argument("--brand", type=str, choices=["feetech", "dynamixel"], required=True, help="Motor brand")
    parser.add_argument("--model", type=str, default="sts3215", help="Motor model used during scanning")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Bus baudrate")
    parser.add_argument("--start-id", type=int, default=1, help="First ID to scan")
    parser.add_argument("--end-id", type=int, default=6, help="Last ID to scan (inclusive)")
    parser.add_argument("--delay", type=float, default=0.5, help="Delay between scans in seconds")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    ids = list(range(args.start_id, args.end_id + 1))
    scan_ids(args.port, args.brand, args.model, args.baudrate, ids, args.delay)

# Running:
# python lerobot/scripts/scan_motor_ids.py --port <port> --brand <servo_brand>