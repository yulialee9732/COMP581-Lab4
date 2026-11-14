#!/usr/bin/env pybricks-micropython

"""EV3 goal navigation with ultrasonic-guided bump recovery."""

import math

from pybricks import ev3brick as brick
from pybricks.ev3devices import GyroSensor, Motor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Button, Port, Stop
from pybricks.tools import StopWatch, wait


# Positions and distances are in mm. Heading is measured CCW from +x.
START_X, START_Y = 500, 0
GOAL_X, GOAL_Y = 2500, 2500
GOAL_RADIUS = 400
START_HEADING = 90  # Face +y before pressing CENTER.

WHEEL_DIAMETER = 55.5
MM_PER_MOTOR_DEG = math.pi * WHEEL_DIAMETER / 360

GOAL_SPEED = 300  # Motor deg/s
WALL_SPEED = 160
WALL_TARGET = 110
WALL_ACQUIRE = 350
WALL_LOST = 450
MIN_WALL_FOLLOW_MM = 250

BACKUP_DISTANCE = 45
BACKUP_LIMIT_MS = 1500
SEARCH_TURN_SPEED = 85  # Motor deg/s
SEARCH_ANGLE_LIMIT = 120
SEARCH_TIME_LIMIT_MS = 10000

WALL_CONFIRM_READINGS = 3
ULTRASONIC_INTERVAL_MS = 50
TIME_LIMIT_MS = 120000
LOOP_MS = 10
GOAL_BEEP_INTERVAL_MS = 1200
DISPLAY_INTERVAL_MS = 250


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_heading(angle):
    return (angle + 180) % 360 - 180


def show_error(error):
    error_type = type(error).__name__
    message = " ".join(str(error).split())

    brick.display.clear()
    brick.display.text("PROGRAM ERROR", (0, 0))
    brick.display.text(error_type[:24], (0, 18))

    line_width = 24
    max_rows = 5
    max_chars = line_width * max_rows
    if len(message) > max_chars:
        message = message[:max_chars - 3] + "..."

    for row in range(max_rows):
        start = row * line_width
        line = message[start:start + line_width]
        if not line:
            break
        brick.display.text(line, (0, 36 + row * 16))


class Robot:
    def __init__(self):
        self.left = Motor(Port.B)
        self.right = Motor(Port.C)
        self.right_touch = TouchSensor(Port.S1)
        self.left_touch = TouchSensor(Port.S4)
        self.gyro = GyroSensor(Port.S2)
        self.ultrasonic = None

    def start(self):
        self.x, self.y = START_X, START_Y

        brick.display.clear()
        brick.display.text(
            "Goal: {:.2f} m".format(self.goal_distance() / 1000),
            (8, 30),
        )
        brick.display.text("Press CENTER", (8, 55))

        while Button.CENTER not in brick.buttons():
            wait(LOOP_MS)
        while Button.CENTER in brick.buttons():
            wait(LOOP_MS)

        # Keep the robot still while zeroing the gyro.
        wait(500)
        self.gyro.reset_angle(0)
        self.heading = START_HEADING
        self.prev_left = self.left.angle()
        self.prev_right = self.right.angle()
        self.timer = StopWatch()
        self.next_display_ms = 0
        self.show_goal_distance()
        print("Started at", self.x, self.y, "heading", self.heading)

    def update_pose(self):
        left = self.left.angle()
        right = self.right.angle()
        distance = (left - self.prev_left + right - self.prev_right) / 2
        distance *= MM_PER_MOTOR_DEG
        self.prev_left, self.prev_right = left, right

        # The gyro angle increases clockwise, while map heading increases CCW.
        heading = START_HEADING - self.gyro.angle()
        midpoint = (self.heading + heading) * math.pi / 360
        self.x += distance * math.cos(midpoint)
        self.y += distance * math.sin(midpoint)
        self.heading = heading
        self.show_goal_distance()
        return distance

    def goal_distance(self):
        dx = GOAL_X - self.x
        dy = GOAL_Y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def show_goal_distance(self, result=None):
        now = self.timer.time()
        if result is None and now < self.next_display_ms:
            return

        self.next_display_ms = now + DISPLAY_INTERVAL_MS
        brick.display.clear()
        brick.display.text(
            "Goal: {:.2f} m".format(self.goal_distance() / 1000),
            (8, 30),
        )

        if result is not None:
            if len(result) <= 24:
                brick.display.text(result, (8, 55))
            else:
                split = result.rfind(" ", 0, 25)
                brick.display.text(result[:split], (8, 55))
                brick.display.text(result[split + 1:], (8, 71))

    def finish_reason(self):
        if self.goal_distance() <= GOAL_RADIUS:
            return "Goal reached"
        if self.timer.time() >= TIME_LIMIT_MS:
            return "Time limit reached"
        return None

    def bumped(self):
        return self.left_touch.pressed() or self.right_touch.pressed()

    def drive(self, left_speed, right_speed):
        self.left.run(int(left_speed))
        self.right.run(int(right_speed))

    def stop(self):
        try:
            self.left.stop(Stop.BRAKE)
        finally:
            self.right.stop(Stop.BRAKE)

    def recover(self, min_turn_deg=0):
        self.stop()
        self.update_pose()
        print("Bump detected. Backing up.")
        brick.sound.beep(300, 120, 55)

        reverse_distance = 0
        maneuver = StopWatch()
        self.drive(-150, -150)
        brick.sound.beep(650, 60, 45)

        while reverse_distance < BACKUP_DISTANCE:
            if self.finish_reason() or maneuver.time() >= BACKUP_LIMIT_MS:
                break
            wait(LOOP_MS)
            reverse_distance -= self.update_pose()

        self.stop()
        reverse_distance -= self.update_pose()
        print("Backup distance", round(reverse_distance), "mm")

        reason = self.finish_reason()
        if reason:
            return reason
        if self.bumped() or reverse_distance < BACKUP_DISTANCE:
            return "Could not back away"

        if self.ultrasonic is None:
            print("Initializing ultrasonic sensor on S3")
            self.ultrasonic = UltrasonicSensor(Port.S3)

        print("Turning right to find the wall")
        start_heading = self.heading
        maneuver.reset()
        next_reading = 0
        wall_readings = 0

        self.drive(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED)

        while True:
            self.update_pose()
            reason = self.finish_reason()
            if reason:
                break

            if self.bumped():
                reason = "Bumped during wall search"
                break

            turned = start_heading - self.heading
            elapsed = maneuver.time()
            if turned >= SEARCH_ANGLE_LIMIT or elapsed >= SEARCH_TIME_LIMIT_MS:
                reason = "Wall search timed out"
                break

            if elapsed >= next_reading:
                clearance = self.ultrasonic.distance()
                next_reading = maneuver.time() + ULTRASONIC_INTERVAL_MS
                print("Search", round(turned), "deg right,", clearance, "mm")

                if turned >= min_turn_deg and 0 < clearance <= WALL_ACQUIRE:
                    wall_readings += 1
                    if wall_readings >= WALL_CONFIRM_READINGS:
                        print("Wall found. Starting wall following.")
                        break
                else:
                    wall_readings = 0

            wait(LOOP_MS)

        self.stop()
        self.update_pose()
        return reason

    def drive_to_goal(self, distance, forward_only=False):
        bearing = math.atan2(GOAL_Y - self.y, GOAL_X - self.x) * 180 / math.pi
        error = wrap_heading(bearing - self.heading)

        if abs(error) > 20 and not forward_only:
            turn = clamp(4 * error, -180, 180)
            self.drive(-turn, turn)
            return

        speed = clamp(2 * (distance - GOAL_RADIUS), 80, GOAL_SPEED)
        steering_fraction = 0.5
        if forward_only:
            speed = min(speed, clamp(GOAL_SPEED - 2 * abs(error), 80, GOAL_SPEED))
            steering_fraction = 0.40

        # Both wheels roll forward, with the inside wheel running slower.
        steering_limit = steering_fraction * speed
        turn = clamp(4 * error, -steering_limit, steering_limit)
        self.drive(speed - turn, speed + turn)

    def follow_wall(self, clearance):
        turn_right = clamp(1.5 * (WALL_TARGET - clearance), 0, 80)
        speed = WALL_SPEED - turn_right / 2
        self.drive(speed + turn_right, speed - turn_right)

    def navigate(self):
        self.start()
        following_wall = False
        forward_only = True
        clear_readings = 0
        next_wall_reading = 0
        next_goal_beep = 0

        while True:
            self.update_pose()
            reason = self.finish_reason()
            if reason:
                return reason

            if self.bumped():
                reason = self.recover(30 if following_wall else 0)
                if reason:
                    return reason

                following_wall = True
                wall_start_x, wall_start_y = self.x, self.y
                clear_readings = 0
                next_wall_reading = 0
                continue

            now = self.timer.time()
            if following_wall:
                if now < next_wall_reading:
                    wait(LOOP_MS)
                    continue

                clearance = self.ultrasonic.distance()
                next_wall_reading = self.timer.time() + ULTRASONIC_INTERVAL_MS
                self.update_pose()

                if self.finish_reason() or self.bumped():
                    self.stop()
                    continue

                # Measure displacement from the start of wall following.
                dx = self.x - wall_start_x
                dy = self.y - wall_start_y
                moved_enough = dx * dx + dy * dy >= MIN_WALL_FOLLOW_MM ** 2

                if moved_enough and clearance > WALL_LOST:
                    clear_readings += 1
                else:
                    clear_readings = 0

                if clear_readings < WALL_CONFIRM_READINGS:
                    self.follow_wall(clearance)
                    wait(LOOP_MS)
                    continue

                print("Left side clear. Resuming toward the goal.")
                following_wall = False
                forward_only = True
                next_goal_beep = self.timer.time()

            self.drive_to_goal(self.goal_distance(), forward_only)
            now = self.timer.time()
            if now >= next_goal_beep:
                brick.sound.beep(1000, 35, 35)
                next_goal_beep = now + GOAL_BEEP_INTERVAL_MS
            wait(LOOP_MS)


try:
    robot = Robot()
    try:
        result = robot.navigate()
    except BaseException:
        # Preserve the original error if braking also raises an error.
        try:
            robot.stop()
        except Exception as stop_error:
            print("Motor stop error:", stop_error)
        raise

    robot.stop()
    print(result)
    robot.show_goal_distance(result)
    brick.sound.beep()
    wait(5000)

except Exception as error:
    print("{}: {}".format(type(error).__name__, error))
    try:
        show_error(error)
        wait(10000)
    except Exception as display_error:
        print("Could not show the error on the EV3:", display_error)
    raise
