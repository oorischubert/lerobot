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
"""
This script configure a single motor at a time to a given ID and baudrate.

Example of usage:
```bash
python lerobot/scripts/configure_motor.py \
  --port /dev/tty.usbmodem585A0080521 \
  --brand feetech \
  --model sts3215 \
  --baudrate 1000000 \
  --ID 1
```
"""

import argparse
import json # servofilter
import os # servofilter

def get_motor_bus_cls(brand: str) -> tuple:
    if brand == "feetech":
        from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
        from lerobot.common.robot_devices.motors.feetech import (
            MODEL_BAUDRATE_TABLE,
            SCS_SERIES_BAUDRATE_TABLE,
            FeetechMotorsBus,
        )

        return FeetechMotorsBusConfig, FeetechMotorsBus, MODEL_BAUDRATE_TABLE, SCS_SERIES_BAUDRATE_TABLE

    elif brand == "dynamixel":
        from lerobot.common.robot_devices.motors.configs import DynamixelMotorsBusConfig
        from lerobot.common.robot_devices.motors.dynamixel import (
            MODEL_BAUDRATE_TABLE,
            X_SERIES_BAUDRATE_TABLE,
            DynamixelMotorsBus,
        )

        return DynamixelMotorsBusConfig, DynamixelMotorsBus, MODEL_BAUDRATE_TABLE, X_SERIES_BAUDRATE_TABLE

    else:
        raise ValueError(
            f"Currently we do not support this motor brand: {brand}. We currently support feetech and dynamixel motors."
        )
        
def configure_servofilter(port, brand, model, motor_idx_des):
    """Set safezone for servo movement to prevent damadge"""
    motor_bus_config_cls, motor_bus_cls, model_baudrate_table, series_baudrate_table = get_motor_bus_cls(
        brand
    )

    # Check if the provided model exists in the model_baud_rate_table
    if model not in model_baudrate_table:
        raise ValueError(
            f"Invalid model '{model}' for brand '{brand}'. Supported models: {list(model_baudrate_table.keys())}"
        )
        
    # Setup motor names, indices, and models
    motor_name = "motor"
    motor_index_arbitrary = motor_idx_des  # Use the motor ID passed via argument
    motor_model = model  # Use the motor model passed via argument

    config = motor_bus_config_cls(port=port, motors={motor_name: (motor_index_arbitrary, motor_model)})

    # Initialize the MotorBus with the correct port and motor configurations
    motor_bus = motor_bus_cls(config=config)

    # Try to connect to the motor bus and handle any connection-specific errors
    try:
        motor_bus.connect()
        print(f"Connected on port {motor_bus.port}")
    except OSError as e:
        print(f"Error occurred when connecting to the motor bus: {e}")
        return
    
    # Motor bus is connected, proceed with the rest of the operations
    try:
        print("Scanning all baudrates and motor indices")
        all_baudrates = set(series_baudrate_table.values())
        motor_index = -1  # Set the motor index to an out-of-range value.

        for baudrate in all_baudrates:
            motor_bus.set_bus_baudrate(baudrate)
            present_ids = motor_bus.find_motor_indices(list(range(1, 10)))

        if motor_index == -1:
            raise ValueError("No motors detected. Please ensure you have at least one motor connected.")

        print(f"Motor index's found at: {motor_index}")
        
        input("Position the motor to the safe point and press Enter...")
        safePoint = motor_bus.read_with_motor_ids(motor_bus.motor_models, motor_idx_des, "Present_Position", num_retry=2)

        input("Now position the motor to the first border and press Enter...")
        border1 = motor_bus.read_with_motor_ids(motor_bus.motor_models, motor_idx_des, "Present_Position", num_retry=2)

        choice = input("Position to the second border and press Enter, or type 'x' and press Enter to skip (one border only): ")
        if choice.strip().lower() == 'x':
            borderMin = border1[0]
            borderMax = border1[0]
        else:
            border2 = motor_bus.read_with_motor_ids(motor_bus.motor_models, motor_idx_des, "Present_Position", num_retry=2)
            borderMin = min(border1[0], border2[0])
            borderMax = max(border1[0], border2[0])
        
        servofilter_path = "servofilter.json"

        motor_data = {
            "safePoint": safePoint[0],
            "borderMin": borderMin,
            "borderMax": borderMax,
        }

        # Load or create the servofilter dictionary
        if os.path.exists(servofilter_path):
            with open(servofilter_path, "r") as f:
                servofilter = json.load(f)
        else:
            servofilter = {}

        # Use motor ID as key
        servofilter[str(motor_idx_des)] = motor_data

        # Save back to file
        with open(servofilter_path, "w") as f:
            json.dump(servofilter, f, indent=4)
        
        print(f"Servo filter data saved to {servofilter_path}.")
        
    except Exception as e:
        print(f"Error occurred during motor configuration: {e}")

    finally:
        motor_bus.disconnect()
        print("Disconnected from motor bus.")
    
         


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, required=True, help="Motors bus port (e.g. dynamixel,feetech)")
    parser.add_argument("--brand", type=str, required=True, help="Motor brand (e.g. dynamixel,feetech)")
    parser.add_argument("--model", type=str, required=True, help="Motor model (e.g. xl330-m077,sts3215)")
    parser.add_argument("--ID", type=int, required=True, help="Desired ID of the current motor (e.g. 1,2,3)")

    args = parser.parse_args()

    configure_servofilter(args.port, args.brand, args.model, args.ID)
