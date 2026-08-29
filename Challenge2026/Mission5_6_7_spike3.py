# Mission5_6_7_spike3.py
# SPIKE App 3 / SPIKE 3 compatible Python conversion
# Converted from the older SPIKE Prime block-style routine.
#
# Paste this into the LEGO Education SPIKE Python editor.
# Uses the newer format:
#   from hub import light_matrix
#   import runloop
#   async def main(): ...
#   runloop.run(main())
#
# DEFAULT PORTS FROM YOUR BLOCK FILE:
#   Drive left motor  = A
#   Drive right motor = B
#   Attachment motor  = C
#   Attachment motor  = D
#
# IMPORTANT:
# Your old project also had a helper routine that seemed to use C/D as color sensors.
# A port cannot be both a motor and a sensor. If you use two color sensors, move them
# to free ports, for example E/F, and update LEFT_SENSOR_PORT / RIGHT_SENSOR_PORT below.

from hub import port, light_matrix, motion_sensor
import runloop
import motor
import motor_pair
import color_sensor

# -----------------------------
# Port setup
# -----------------------------
LEFT_DRIVE_PORT = port.A
RIGHT_DRIVE_PORT = port.B
ATTACHMENT_C_PORT = port.C
ATTACHMENT_D_PORT = port.D

# Change these if you use line squaring.
# Recommended: put color sensors on E/F if C/D are attachment motors.
LEFT_SENSOR_PORT = port.E
RIGHT_SENSOR_PORT = port.F

DRIVE_PAIR = motor_pair.PAIR_1

# -----------------------------
# Robot constants
# -----------------------------
WHEEL_CIRCUMFERENCE_CM = 19.59
DEGREES_PER_CM = 360 / WHEEL_CIRCUMFERENCE_CM

# Reflection thresholds. Tune these on your real mat.
BLACK_REFLECT = 32
WHITE_REFLECT = 80

# Old block speed was percent-ish. SPIKE 3 motor velocity is degrees/second.
# 100% roughly maps to 1000 deg/sec for large motors.
def pct_to_velocity(speed_pct):
    return int(speed_pct * 10)


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def yaw_degrees():
    # SPIKE 3 tilt_angles returns decidegrees: yaw, pitch, roll.
    yaw, pitch, roll = motion_sensor.tilt_angles()
    return yaw / 10


def reset_yaw():
    motion_sensor.reset_yaw(0)


async def stop_drive():
    motor_pair.stop(DRIVE_PAIR, stop=motor.BRAKE)
    await runloop.sleep_ms(100)


async def reset_drive_motors():
    # Reset relative motor positions if supported by your SPIKE firmware.
    # If your app complains, comment out these two lines.
    try:
        motor.reset_relative_position(LEFT_DRIVE_PORT, 0)
        motor.reset_relative_position(RIGHT_DRIVE_PORT, 0)
    except Exception:
        pass
    await runloop.sleep_ms(50)


# -----------------------------
# Basic drive helpers
# -----------------------------
async def move_straight_accurately(cm, speed_pct):
    """
    Equivalent behavior to your original block's "Move straight accurately".

    Note: the original block had gyro correction multiplied by 0, so it did not
    actually correct heading. This version uses the built-in motor-pair distance
    move, which is the safest direct SPIKE 3 equivalent.
    """
    degrees = int(abs(cm) * DEGREES_PER_CM)
    velocity = pct_to_velocity(speed_pct)
    await motor_pair.move_for_degrees(
        DRIVE_PAIR,
        degrees,
        0,
        velocity=velocity,
        stop=motor.BRAKE,
    )
    await runloop.sleep_ms(100)


async def move_straight_gyro(cm, speed_pct, gain=12):
    """
    Improved straight drive using yaw correction.
    Use this after the mission works, then tune gain.

    gain suggestion:
      8-12 = gentle correction
      14-20 = stronger correction
    """
    await reset_drive_motors()
    reset_yaw()

    target_degrees = int(abs(cm) * DEGREES_PER_CM)
    base_velocity = pct_to_velocity(speed_pct)
    direction = 1 if base_velocity >= 0 else -1

    while True:
        try:
            right_pos = abs(motor.relative_position(RIGHT_DRIVE_PORT))
        except Exception:
            # Fallback: if relative_position is unavailable, use built-in move.
            await move_straight_accurately(cm, speed_pct)
            return

        if right_pos >= target_degrees:
            break

        correction = int(yaw_degrees() * gain)
        left_velocity = base_velocity - correction * direction
        right_velocity = base_velocity + correction * direction
        left_velocity = clamp(left_velocity, -1000, 1000)
        right_velocity = clamp(right_velocity, -1000, 1000)

        motor_pair.move_tank(DRIVE_PAIR, left_velocity, right_velocity)
        await runloop.sleep_ms(10)

    await stop_drive()


async def move_gyro_straight_for_seconds(speed_pct, seconds):
    """
    Equivalent to old helper: drive straight for time.
    Preserves old behavior with no real gyro correction.
    """
    velocity = pct_to_velocity(speed_pct)
    motor_pair.move_tank(DRIVE_PAIR, velocity, velocity)
    await runloop.sleep_ms(int(seconds * 1000))
    await stop_drive()


async def turn_right(angle, speed_pct=20):
    reset_yaw()
    velocity = abs(pct_to_velocity(speed_pct))
    motor_pair.move_tank(DRIVE_PAIR, velocity, -velocity)

    while yaw_degrees() < angle:
        await runloop.sleep_ms(5)

    await stop_drive()


async def turn_left(angle, speed_pct=30):
    """
    Angle should normally be negative, for example -24.
    """
    reset_yaw()
    velocity = abs(pct_to_velocity(speed_pct))
    motor_pair.move_tank(DRIVE_PAIR, -velocity, velocity)

    while yaw_degrees() > angle:
        await runloop.sleep_ms(5)

    await stop_drive()


# -----------------------------
# Attachment helpers
# -----------------------------
async def motor_turn_degrees(motor_port, degrees, speed_pct=20):
    velocity = pct_to_velocity(speed_pct)
    if degrees < 0 and velocity > 0:
        velocity = -velocity
    elif degrees > 0 and velocity < 0:
        velocity = -velocity

    await motor.run_for_degrees(
        motor_port,
        int(abs(degrees)),
        velocity,
        stop=motor.BRAKE,
    )
    await runloop.sleep_ms(100)


async def move_right_attachment(up_or_down, rotations, speed_pct=20):
    degrees = int(rotations * 360)
    if up_or_down < 0:
        degrees = -degrees
    await motor_turn_degrees(ATTACHMENT_C_PORT, degrees, speed_pct)


# -----------------------------
# Optional line squaring helper
# -----------------------------
async def square_to_black_line(speed_pct=15, retry=2):
    """
    Optional helper if your robot has two color sensors on LEFT_SENSOR_PORT and RIGHT_SENSOR_PORT.
    It drives forward slowly until each sensor sees black, stopping each side independently.
    """
    velocity = abs(pct_to_velocity(speed_pct))

    for attempt in range(retry):
        left_done = False
        right_done = False

        motor.run(LEFT_DRIVE_PORT, velocity)
        motor.run(RIGHT_DRIVE_PORT, velocity)

        while not (left_done and right_done):
            left_reflect = color_sensor.reflection(LEFT_SENSOR_PORT)
            right_reflect = color_sensor.reflection(RIGHT_SENSOR_PORT)

            if left_reflect <= BLACK_REFLECT and not left_done:
                motor.stop(LEFT_DRIVE_PORT, stop=motor.BRAKE)
                left_done = True

            if right_reflect <= BLACK_REFLECT and not right_done:
                motor.stop(RIGHT_DRIVE_PORT, stop=motor.BRAKE)
                right_done = True

            await runloop.sleep_ms(5)

        if attempt < retry - 1:
            motor_pair.move_tank(DRIVE_PAIR, -velocity, -velocity)
            await runloop.sleep_ms(300)
            await stop_drive()


# -----------------------------
# Mission 5 / 6 / 7 sequence
# -----------------------------
async def mission_5_6_7():
    await light_matrix.write("567")

    # Pair left/right drive motors.
    motor_pair.pair(DRIVE_PAIR, LEFT_DRIVE_PORT, RIGHT_DRIVE_PORT)

    # Original mission sequence from your blocks.
    await move_straight_accurately(66, -40)
    await turn_right(25.5)

    await motor_turn_degrees(ATTACHMENT_C_PORT, -360, 20)
    await move_straight_accurately(3, -30)
    await motor_turn_degrees(ATTACHMENT_C_PORT, 120, 20)
    await motor_turn_degrees(ATTACHMENT_D_PORT, 350, 20)
    await motor_turn_degrees(ATTACHMENT_C_PORT, 140, 20)

    await move_straight_accurately(1, -30)
    await turn_left(-11)
    await turn_right(12)
    await move_straight_accurately(11, 30)
    await turn_left(-24)
    await move_straight_accurately(20, -30)
    await turn_left(-20)
    await move_straight_accurately(2, 30)
    await turn_right(20)
    await turn_left(-20)
    await move_straight_accurately(80, 100)

    await motor_turn_degrees(ATTACHMENT_D_PORT, -148, 20)
    await light_matrix.write("OK")


async def main():
    await mission_5_6_7()


runloop.run(main())
