from controller import Supervisor

# ==========================================================
# Delivery Robot Controller
# Simple FSM with Safety / Fail-safe Mechanism
#
# Logic:
# IDLE -> SEARCH
# SEARCH / NAVIGATE: move forward
# If obstacle detected: turn left 90 degrees
# Then check front:
#   if clear -> continue moving forward
#   if blocked -> turn left 180 degrees
# Camera detects red delivery zone
# Robot stops when it reaches the target delivery zone
# Safety:
#   if runtime is too long -> SAFE_STOP
#   if too many avoidance attempts -> SAFE_STOP
# ==========================================================

robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())

print("Delivery robot FSM controller started")

# ==========================================================
# Motors
# ==========================================================

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ==========================================================
# Front Distance Sensors Only
# ==========================================================

ps0 = robot.getDevice("ps0")  # front right
ps7 = robot.getDevice("ps7")  # front left

ps0.enable(TIME_STEP)
ps7.enable(TIME_STEP)

# ==========================================================
# Camera
# ==========================================================

camera = None

try:
    camera = robot.getDevice("camera")
    camera.enable(TIME_STEP)
    print("Camera enabled")
except:
    print("Camera not found")

# ==========================================================
# Supervisor position
# ==========================================================

self_node = robot.getSelf()

# ==========================================================
# Parameters
# ==========================================================

FORWARD_SPEED = 3.0
TURN_SPEED = 3.0

# Normal obstacle detection while moving forward
FORWARD_OBSTACLE_THRESHOLD = 300.0

# More sensitive check after turning left 90 degrees
CHECK_FRONT_OBSTACLE_THRESHOLD = 100.0

# Turn durations
TURN_90_DURATION = 0.736
TURN_180_DURATION = 1.472

# Wait after turning so sensor values become stable
CHECK_WAIT_DURATION = 0.50

# Camera red colour detection
RED_THRESHOLD = 100
RED_RATIO_THRESHOLD = 0.006

# Target delivery zone position
# Change these if your red ground delivery zone has a different translation.
TARGET_X = 0.40
TARGET_Y = 0.00
TARGET_RADIUS = 0.15

# Safety / fail-safe settings
MAX_RUNTIME = 60.0
MAX_AVOID_ATTEMPTS = 8

# ==========================================================
# FSM States
# ==========================================================

IDLE = "IDLE"
SEARCH = "SEARCH"
NAVIGATE = "NAVIGATE"
TURN_LEFT_90 = "TURN_LEFT_90"
WAIT_BEFORE_CHECK = "WAIT_BEFORE_CHECK"
CHECK_FRONT = "CHECK_FRONT"
TURN_180 = "TURN_180"
TASK_COMPLETE = "TASK_COMPLETE"
SAFE_STOP = "SAFE_STOP"

state = IDLE

turn_start_time = 0.0
check_start_time = 0.0
avoid_attempts = 0

safe_stop_reason = ""

# ==========================================================
# Motor Functions
# ==========================================================

def move_forward():
    left_motor.setVelocity(FORWARD_SPEED)
    right_motor.setVelocity(FORWARD_SPEED)


def turn_left():
    left_motor.setVelocity(-TURN_SPEED)
    right_motor.setVelocity(TURN_SPEED)


def stop_robot():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

# ==========================================================
# Sensor Functions
# ==========================================================

def get_front_sensor_value():
    front_right = ps0.getValue()
    front_left = ps7.getValue()
    front = max(front_left, front_right)
    return front, front_left, front_right


def obstacle_detected_while_moving(front):
    return front > FORWARD_OBSTACLE_THRESHOLD


def obstacle_detected_after_turn(front):
    return front > CHECK_FRONT_OBSTACLE_THRESHOLD

# ==========================================================
# Camera Red Target Detection
# ==========================================================

def detect_red_target():
    if camera is None:
        return False, 0.0

    image = camera.getImage()

    if image is None:
        return False, 0.0

    width = camera.getWidth()
    height = camera.getHeight()

    red_pixels = 0
    sampled_pixels = 0

    for x in range(0, width, 4):
        for y in range(0, height, 4):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            sampled_pixels += 1

            if r > RED_THRESHOLD and r > g * 1.3 and r > b * 1.3:
                red_pixels += 1

    if sampled_pixels == 0:
        return False, 0.0

    red_ratio = red_pixels / sampled_pixels

    if red_ratio > RED_RATIO_THRESHOLD:
        return True, red_ratio

    return False, red_ratio

# ==========================================================
# Target Zone Check
# ==========================================================

def get_robot_position():
    position = self_node.getPosition()
    robot_x = position[0]
    robot_y = position[1]
    return robot_x, robot_y


def is_on_target_zone():
    robot_x, robot_y = get_robot_position()

    dx = robot_x - TARGET_X
    dy = robot_y - TARGET_Y

    distance_to_target = (dx * dx + dy * dy) ** 0.5

    on_target = distance_to_target <= TARGET_RADIUS

    return on_target, distance_to_target, robot_x, robot_y

# ==========================================================
# Safety Check
# ==========================================================

def safety_check(current_time):
    global safe_stop_reason

    if current_time > MAX_RUNTIME:
        safe_stop_reason = "Maximum runtime exceeded"
        return True

    if avoid_attempts > MAX_AVOID_ATTEMPTS:
        safe_stop_reason = "Too many obstacle avoidance attempts"
        return True

    return False

# ==========================================================
# Main Loop
# ==========================================================

while robot.step(TIME_STEP) != -1:
    current_time = robot.getTime()

    front, front_left, front_right = get_front_sensor_value()
    target_detected, red_ratio = detect_red_target()
    on_target, distance_to_target, robot_x, robot_y = is_on_target_zone()

    # ======================================================
    # FSM Decision Logic
    # ======================================================

    if state == IDLE:
        state = SEARCH

    elif state == TASK_COMPLETE:
        state = TASK_COMPLETE

    elif state == SAFE_STOP:
        state = SAFE_STOP

    elif safety_check(current_time):
        state = SAFE_STOP

    elif on_target:
        state = TASK_COMPLETE

    elif state == SEARCH:
        if obstacle_detected_while_moving(front):
            avoid_attempts += 1
            state = TURN_LEFT_90
            turn_start_time = current_time
        elif target_detected:
            state = NAVIGATE
        else:
            state = SEARCH

    elif state == NAVIGATE:
        if obstacle_detected_while_moving(front):
            avoid_attempts += 1
            state = TURN_LEFT_90
            turn_start_time = current_time
        else:
            state = NAVIGATE

    elif state == TURN_LEFT_90:
        if current_time - turn_start_time >= TURN_90_DURATION:
            state = WAIT_BEFORE_CHECK
            check_start_time = current_time
        else:
            state = TURN_LEFT_90

    elif state == WAIT_BEFORE_CHECK:
        if current_time - check_start_time >= CHECK_WAIT_DURATION:
            state = CHECK_FRONT
        else:
            state = WAIT_BEFORE_CHECK

    elif state == CHECK_FRONT:
        if obstacle_detected_after_turn(front):
            avoid_attempts += 1
            state = TURN_180
            turn_start_time = current_time
        elif target_detected:
            state = NAVIGATE
        else:
            state = SEARCH

    elif state == TURN_180:
        if current_time - turn_start_time >= TURN_180_DURATION:
            state = SEARCH
        else:
            state = TURN_180

    # ======================================================
    # FSM Action Logic
    # ======================================================

    if state == SEARCH:
        move_forward()

    elif state == NAVIGATE:
        move_forward()

    elif state == TURN_LEFT_90:
        turn_left()

    elif state == WAIT_BEFORE_CHECK:
        stop_robot()

    elif state == CHECK_FRONT:
        stop_robot()

    elif state == TURN_180:
        turn_left()

    elif state == TASK_COMPLETE:
        stop_robot()

    elif state == SAFE_STOP:
        stop_robot()

    # ======================================================
    # Console Output
    # ======================================================

    print(
        f"Time: {current_time:.2f} | "
        f"State: {state} | "
        f"Front: {front:.2f} | "
        f"FrontLeft: {front_left:.2f} | "
        f"FrontRight: {front_right:.2f} | "
        f"ForwardThreshold: {FORWARD_OBSTACLE_THRESHOLD:.1f} | "
        f"CheckThreshold: {CHECK_FRONT_OBSTACLE_THRESHOLD:.1f} | "
        f"TargetDetected: {target_detected} | "
        f"RedRatio: {red_ratio:.3f} | "
        f"RobotX: {robot_x:.2f} | "
        f"RobotY: {robot_y:.2f} | "
        f"DistanceToTarget: {distance_to_target:.3f} | "
        f"OnTarget: {on_target} | "
        f"AvoidAttempts: {avoid_attempts} | "
        f"SafeStopReason: {safe_stop_reason}"
    )