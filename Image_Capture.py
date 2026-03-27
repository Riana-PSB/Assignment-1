from controller import Robot, Motor, DistanceSensor, Camera

# ================= CONSTANTS =================
MAX_SPEED = 6.28
MULTIPLIER = 0.3

OBSTACLE_DISTANCE = 0.02
CAPTURE_DISTANCE = 0.01

# Deer RGB
DEER_R = 74
DEER_G = 76
DEER_B = 83
TOLERANCE = 40

# Color detection
DOMINANCE_MARGIN = 40
MIN_INTENSITY = 120

# ================= DISTANCE =================
def get_distance_values(sensors, values):
    for i in range(8):
        val = sensors[i].getValue() / 4096.0
        values[i] = min(val, 1.0)

def front_obstacle(values):
    return (values[0] + values[7]) / 2.0 > OBSTACLE_DISTANCE

def obstacle_left(values):
    return values[5] > OBSTACLE_DISTANCE or values[6] > OBSTACLE_DISTANCE

def obstacle_right(values):
    return values[1] > OBSTACLE_DISTANCE or values[2] > OBSTACLE_DISTANCE

def close_enough(values):
    return (values[0] + values[7]) / 2.0 > CAPTURE_DISTANCE

# ================= MOVEMENT =================
def move_forward(lm, rm):
    lm.setVelocity(MAX_SPEED * MULTIPLIER)
    rm.setVelocity(MAX_SPEED * MULTIPLIER)

def move_slow(lm, rm):
    lm.setVelocity(MAX_SPEED * 0.1)
    rm.setVelocity(MAX_SPEED * 0.1)

def move_backward(lm, rm, robot, timestep):
    lm.setVelocity(-MAX_SPEED * MULTIPLIER)
    rm.setVelocity(-MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)

def turn_left(lm, rm, robot, timestep):
    lm.setVelocity(-MAX_SPEED * MULTIPLIER)
    rm.setVelocity(MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)

def turn_right(lm, rm, robot, timestep):
    lm.setVelocity(MAX_SPEED * MULTIPLIER)
    rm.setVelocity(-MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)

def wait(robot, timestep, sec):
    start = robot.getTime()
    while robot.getTime() < start + sec:
        robot.step(timestep)

# ================= CAMERA =================
def get_center_rgb(camera):
    w = camera.getWidth()
    h = camera.getHeight()
    img = camera.getImage()

    x = int(w / 2)
    y = int(h / 2)

    r = camera.imageGetRed(img, w, x, y)
    g = camera.imageGetGreen(img, w, x, y)
    b = camera.imageGetBlue(img, w, x, y)

    return r, g, b

# ================= DEER DETECTION =================
def is_deer(r, g, b):
    return (abs(r - DEER_R) < TOLERANCE and
            abs(g - DEER_G) < TOLERANCE and
            abs(b - DEER_B) < TOLERANCE)

# ================= COLOR DETECTION =================
def detect_color(r, g, b):
    if r > MIN_INTENSITY and r - max(g, b) > DOMINANCE_MARGIN:
        return "Red"
    elif g > MIN_INTENSITY and g - max(r, b) > DOMINANCE_MARGIN:
        return "Green"
    elif b > MIN_INTENSITY and b - max(r, g) > DOMINANCE_MARGIN:
        return "Blue"
    return None

# ================= IMAGE CAPTURE =================
def capture_image(camera):
    filename = "deer_capture.png"
    camera.saveImage(filename, 100)
    print("Image saved:", filename)

def capture_stable(robot, camera, timestep):
    for _ in range(10):
        robot.step(timestep)
    capture_image(camera)

# ================= MAIN =================
def run_robot(robot):

    timestep = int(robot.getBasicTimeStep())

    # Sensors
    names = ["ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7"]
    sensors = []
    values = [0.0]*8

    for n in names:
        s = robot.getDevice(n)
        s.enable(timestep)
        sensors.append(s)

    # Camera
    camera = robot.getDevice("camera")
    camera.enable(timestep)

    # Motors
    lm = robot.getDevice("left wheel motor")
    rm = robot.getDevice("right wheel motor")

    lm.setPosition(float('inf'))
    rm.setPosition(float('inf'))

    lm.setVelocity(0)
    rm.setVelocity(0)

    encountered_colors = set()
    captured = False

    # ================= LOOP =================
    while robot.step(timestep) != -1:

        get_distance_values(sensors, values)
        r, g, b = get_center_rgb(camera)

        # ===== 1. OBSTACLE AVOIDANCE =====
        if front_obstacle(values):

            move_backward(lm, rm, robot, timestep)

            if obstacle_left(values):
                turn_right(lm, rm, robot, timestep)
            else:
                turn_left(lm, rm, robot, timestep)

        # ===== 2. DEER BEHAVIOR =====
        elif is_deer(r, g, b):

            # BEFORE capture → approach
            if not captured:
                print("🦌 DEER DETECTED")

                if not close_enough(values):
                    move_slow(lm, rm)
                else:
                    lm.setVelocity(0)
                    rm.setVelocity(0)

                    capture_stable(robot, camera, timestep)
                    captured = True

                    print("📸 Deer captured. Avoiding deer now...")

            # AFTER capture → treat like obstacle
            else:
                move_backward(lm, rm, robot, timestep)
                turn_left(lm, rm, robot, timestep)

        # ===== 3. COLOR DETECTION =====
        else:
            color = detect_color(r, g, b)

            if color and color not in encountered_colors:
                encountered_colors.add(color)
                print(f"🎨 Detected: {color}")
                print(f"Seen so far: {encountered_colors}")

            # ===== 4. WANDER =====
            move_forward(lm, rm)


# ================= ENTRY =================
if __name__ == "__main__":
    robot = Robot()
    run_robot(robot)