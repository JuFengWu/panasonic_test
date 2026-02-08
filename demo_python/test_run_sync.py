"""Python demo for high-level API usage (expected binding shape).

Adjust the import module name to match the actual Python package when it exists.
"""

import time

# TODO: replace "motorcode" with your actual binding module name.
from motorcode import MotionSystem, MotorModel, MotorModes


def on_cycle():
    """Realtime callback called by the control loop."""
    # Read status periodically
    global on_cycle_counter
    on_cycle_counter += 1
    if on_cycle_counter % 50 == 0:
        m1 = sys.motor(1)
        print("Current Position:", m1.get_current_position())
        print("Error State:", m1.get_error_code())

    # Simple CSP example (set a position once after some cycles)
    if on_cycle_counter == 100:
        sys.motor(1).set_target_position(30.0)
        # request shutdown after the move command
        return True

    return False


on_cycle_counter = 0
sys = None


def main():
    ifname = "enp3s0"  # network interface name
    model = MotorModel.PanasonicA6MotorType
    mode = MotorModes.CSP_Mode

    global sys
    sys = MotionSystem(model, ifname)

    motor_count = 1
    cycle_period_ms = 1
    if not sys.start_connect(motor_count, cycle_period_ms, mode):
        raise RuntimeError("start_connect failed")

    sys.set_cycle_log_enabled(True)

    sys.set_callback(on_cycle)

    if not sys.run_async_io_only():
        sys.close()
        raise RuntimeError("run_async_io_only failed")

    m1 = sys.motor(1)
    m1.servo_on()

    # Start control loop (will call on_cycle)
    sys.start_control_loop()

    # Give it time to run; adjust as needed for your environment
    time.sleep(2)

    m1.servo_off()
    sys.close()


if __name__ == "__main__":
    main()
