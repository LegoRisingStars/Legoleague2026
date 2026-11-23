# FLL_spike3_drive_routines.py
# SPIKE App 3 compatible Python drive helper routines
# Uses the newer async style:
#   from hub import light_matrix
#   import runloop
#   async def main(): ...
#   runloop.run(main())
#
# Assumed robot setup:
#   Left drive motor  = port.A
#   Right drive motor = port.B
#   Wheel size        = 62.4 x 20 tire, approx 19.59 cm circumference
#
# IMPORTANT TUNING:
#   1. If forward goes backward, swap A/B or change signs in move_tank.
#   2. TURN_FACTOR depends on your wheel spacing. Tune it on your robot.
#   3. For FLL, always test on the real mat with your battery fully charged.

from hub import port, light_matrix, motion_sensor
import runloop
import motor
import motor_pair

# -----------------------------
# Ports and constants
# -----------------------------
LEFT_DRIVE_PORT = port.A
RIGHT_DRIVE_PORT = port.B
DRIVE_PAIR = motor_pair.PAIR_1

# 62.4 mm wheel circumference = pi * 6.24 cm = about 19.59 cm
WHEEL_CIRCUMFERENCE_CM = 19.59
DEGREES_PER_CM = 360 / WHEEL_CIRCUMFERENCE_CM

# Tune this for your robot.
# For an in-place turn, each wheel's motor degrees needed per 1 robot degree.
# Start around 2.0 to 2.8 for many SPIKE Prime FLL bases.
TURN_FACTOR = 2.35

MAX_VELOCITY = 1050   # Large motor max is about 1050 deg/sec


# -----------------------------
# Small utility helpers
# -----------------------------
def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def speed_to_velocity(speed_pct):
    """
    Convert a kid-friendly block speed, like 40 or -40, into SPIKE 3 motor velocity.
    40 means about 40% speed. -40 means reverse.
    """
    return clamp(int(speed_pct * 10), -MAX_VELOCITY, MAX_VELOCITY)


def cm_to_motor_degrees(cm):
    return int(abs(cm) * DEGREES_PER_CM)


def yaw_degrees():
    """
    SPIKE 3 motion_sensor.tilt_angles()[0] gives yaw in decidegrees.
    900 = 90.0 degrees, so divide by 10.
    """
    yaw, pitch, roll = motion_sensor.tilt_angles()
    return yaw / 10


def reset_yaw():
    motion_sensor.reset_yaw(0)


async def pair_drive_motors():
    motor_pair.pair(DRIVE_PAIR, LEFT_DRIVE_PORT, RIGHT_DRIVE_PORT)
    await runloop.sleep_ms(100)


async def stop_drive():
    motor_pair.stop(DRIVE_PAIR, stop=motor.BRAKE)
    await runloop.sleep_ms(100)


async def reset_drive_positions():
    motor.reset_relative_position(LEFT_DRIVE_PORT, 0)
    motor.reset_relative_position(RIGHT_DRIVE_PORT, 0)
    await runloop.sleep_ms(50)


# =============================================================
# ROUTINE 1: Move straight accurately
# Equivalent custom block:
#   move straight accurately distance_cm speed
# Example:
#   await move_straight_accurately(50, 40)
# =============================================================
async def move_straight_accurately(distance_cm, speed_pct=40, correction_gain=10):
    """
    Drives straight using motor distance plus gyro correction.

    Parameters:
      distance_cm      positive number of centimeters to move
      speed_pct        positive = forward, negative = backward
      correction_gain  higher = stronger straightening correction

    Good starting values:
      speed_pct = 25 to 45 for accurate FLL runs
      correction_gain = 8 to 14
    """
    await reset_drive_positions()
    reset_yaw()

    target_degrees = cm_to_motor_degrees(distance_cm)
    base_velocity = speed_to_velocity(speed_pct)

    # Avoid zero-speed mistakes.
    if base_velocity == 0:
        await stop_drive()
        return

    direction = 1 if base_velocity > 0 else -1

    while True:
        left_pos = abs(motor.relative_position(LEFT_DRIVE_PORT))
        right_pos = abs(motor.relative_position(RIGHT_DRIVE_PORT))
        average_pos = (left_pos + right_pos) / 2

        if average_pos >= target_degrees:
            break

        # If yaw is positive, robot has drifted right; correct by slowing left / speeding right.
        error = yaw_degrees()
        correction = int(error * correction_gain)

        left_velocity = base_velocity - correction * direction
        right_velocity = base_velocity + correction * direction

        left_velocity = clamp(left_velocity, -MAX_VELOCITY, MAX_VELOCITY)
        right_velocity = clamp(right_velocity, -MAX_VELOCITY, MAX_VELOCITY)

        motor_pair.move_tank(DRIVE_PAIR, left_velocity, right_velocity)
        await runloop.sleep_ms(10)

    await stop_drive()


# =============================================================
# ROUTINE 2: Move reverse
# Equivalent custom block:
#   move reverse distance_cm speed
# Example:
#   await move_reverse(30, 35)
# =============================================================
async def move_reverse(distance_cm, speed_pct=35):
    """
    Reverse version of move_straight_accurately.
    Use a positive speed_pct; the routine makes it negative internally.
    """
    await move_straight_accurately(distance_cm, -abs(speed_pct))


# =============================================================
# ROUTINE 3: Move left with degrees parameter
# Equivalent custom block:
#   move left degrees speed
# Example:
#   await move_left_degrees(90, 30)
# =============================================================
async def move_left_degrees(robot_degrees, speed_pct=30):
    """
    Turns robot left in place by robot_degrees.

    This uses wheel motor degrees, not gyro. It is simple and close to how
    a block turn-for-degrees routine usually works.

    Tune TURN_FACTOR until this is accurate:
      If 90 turns only 80 degrees, increase TURN_FACTOR.
      If 90 turns 100 degrees, decrease TURN_FACTOR.
    """
    turn_motor_degrees = int(abs(robot_degrees) * TURN_FACTOR)
    velocity = abs(speed_to_velocity(speed_pct))

    # Left turn: left wheel backward, right wheel forward.
    await motor_pair.move_tank_for_degrees(
        DRIVE_PAIR,
        turn_motor_degrees,
        -velocity,
        velocity,
        stop=motor.BRAKE,
    )
    await runloop.sleep_ms(100)


# =============================================================
# ROUTINE 4: Move left using gyro
# Equivalent custom block:
#   move left degrees using gyro
# Example:
#   await move_left(90)
# =============================================================
async def move_left(robot_degrees=90, speed_pct=25):
    """
    More accurate left turn using the hub gyro/yaw sensor.
    This is usually better for FLL than motor-degrees-only turning.
    """
    reset_yaw()
    velocity = abs(speed_to_velocity(speed_pct))

    # Left turn: negative steering / tank left backward, right forward.
    motor_pair.move_tank(DRIVE_PAIR, -velocity, velocity)

    # Left turn normally makes yaw negative on many SPIKE setups.
    target = -abs(robot_degrees)
    while yaw_degrees() > target:
        await runloop.sleep_ms(5)

    await stop_drive()


# =============================================================
# Optional: right turn too, useful for testing symmetry
# =============================================================
async def move_right(robot_degrees=90, speed_pct=25):
    reset_yaw()
    velocity = abs(speed_to_velocity(speed_pct))

    motor_pair.move_tank(DRIVE_PAIR, velocity, -velocity)

    target = abs(robot_degrees)
    while yaw_degrees() < target:
        await runloop.sleep_ms(5)

    await stop_drive()


# -----------------------------
# Test / demo program
# -----------------------------
async def main():
    await light_matrix.write("FLL")
    await pair_drive_motors()

    # Uncomment one test at a time.
    await move_straight_accurately(30, 35)
    await runloop.sleep_ms(500)

    await move_left(90, 25)
    await runloop.sleep_ms(500)

    await move_reverse(20, 35)
    await runloop.sleep_ms(500)

    # Alternative left turn using motor degrees instead of gyro:
    # await move_left_degrees(90, 30)

    await light_matrix.write("OK")


runloop.run(main())
