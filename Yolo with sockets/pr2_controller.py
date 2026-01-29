from controller import Supervisor, Keyboard, Camera, Lidar, Motor
import math, cv2, socket
import numpy as np

TIME_STEP = 16
MAX_WHEEL_SPEED = 6.0
ROTATE_SPEED = 2.5
OBJECT_DISTANCE_THRESHOLD = 2.0
FRONT_FOV = 0.5
FRONT_STABLE_FRAMES = 8
DISTANCE_EPS = 0.05  

robot = Supervisor()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

wheel_motors = []
rotation_motors = []
right_head_cam = None
base_lidar = None

last_state = None
front_stable_counter = 0
qr_frame_skip = 0
QR_SKIP_N = 6

last_valid_qr = None
current_qr = None
last_printed_dist = None

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5005))


def cluster_lidar(ranges, max_gap=0.12):
    clusters = []
    current = []

    for r in ranges:
        if math.isinf(r) or r > 10:
            if current:
                clusters.append(current)
                current = []
        else:
            if not current:
                current = [r]
            else:
                if abs(r - current[-1]) < max_gap:
                    current.append(r)
                else:
                    clusters.append(current)
                    current = [r]

    if current:
        clusters.append(current)

    return clusters


def get_front_object_distance(ranges):
    c = len(ranges)//2
    w = int(len(ranges)*FRONT_FOV/2)
    front = ranges[c-w:c+w]

    clusters = cluster_lidar(front)
    clusters = [cl for cl in clusters if len(cl) > 8]

    if not clusters:
        return None

    return min(min(cl) for cl in clusters)


def send_frame(camera):
    img = camera.getImage()
    w = camera.getWidth()
    h = camera.getHeight()

    image = np.frombuffer(img, np.uint8).reshape((h, w, 4))
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    _, enc = cv2.imencode(".jpg", image)
    data = enc.tobytes()

    sock.sendall(len(data).to_bytes(4, "big"))
    sock.sendall(data)

    size_data = sock.recv(4)
    size = int.from_bytes(size_data, "big")

    buf = b""
    while len(buf) < size:
        buf += sock.recv(size - len(buf))

    return buf.decode()


def initialize_devices():
    global right_head_cam, base_lidar

    wheel_names = [
        "fl_caster_l_wheel_joint","fl_caster_r_wheel_joint",
        "fr_caster_l_wheel_joint","fr_caster_r_wheel_joint",
        "bl_caster_l_wheel_joint","bl_caster_r_wheel_joint",
        "br_caster_l_wheel_joint","br_caster_r_wheel_joint"
    ]
    for name in wheel_names:
        m = robot.getDevice(name)
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        wheel_motors.append(m)

    rotation_names = [
        "fl_caster_rotation_joint",
        "fr_caster_rotation_joint",
        "bl_caster_rotation_joint",
        "br_caster_rotation_joint"
    ]
    for name in rotation_names:
        rotation_motors.append(robot.getDevice(name))

    right_head_cam = robot.getDevice("wide_stereo_r_stereo_camera_sensor")
    right_head_cam.enable(TIME_STEP)

    base_lidar = robot.getDevice("base_laser")
    base_lidar.enable(TIME_STEP)
    base_lidar.enablePointCloud()


def set_wheels(speed):
    for m in wheel_motors:
        m.setVelocity(speed)


def stop_wheels():
    set_wheels(0.0)


def set_caster_angles(fl, fr, bl, br):
    rotation_motors[0].setPosition(fl)
    rotation_motors[1].setPosition(fr)
    rotation_motors[2].setPosition(bl)
    rotation_motors[3].setPosition(br)


initialize_devices()

print("Robot           : PR2")
print("RoboticSystem   : Warehouse")
print("Process         : Idle")


while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()
    rotating = False

    if key == Keyboard.UP:
        set_caster_angles(0,0,0,0)
        set_wheels(MAX_WHEEL_SPEED)
    elif key == Keyboard.DOWN:
        set_caster_angles(0,0,0,0)
        set_wheels(-MAX_WHEEL_SPEED)
    elif key == Keyboard.LEFT:
        rotating = True
        set_caster_angles(3*math.pi/4, math.pi/4, -3*math.pi/4, -math.pi/4)
        set_wheels(ROTATE_SPEED)
    elif key == Keyboard.RIGHT:
        rotating = True
        set_caster_angles(3*math.pi/4, math.pi/4, -3*math.pi/4, -math.pi/4)
        set_wheels(-ROTATE_SPEED)
    else:
        stop_wheels()

    front_dist = get_front_object_distance(base_lidar.getRangeImage())

    if front_dist is not None and front_dist < OBJECT_DISTANCE_THRESHOLD and not rotating:
        front_stable_counter += 1
    else:
        front_stable_counter = 0

    front_close = front_stable_counter >= FRONT_STABLE_FRAMES

    if not front_close:
        current_qr = None
        last_printed_dist = None

    vision_result = None
    if front_close:
        qr_frame_skip += 1
        if qr_frame_skip % QR_SKIP_N == 0:
            vision_result = send_frame(right_head_cam)

    if vision_result:
        parts = vision_result.split(";")
        human_flag = parts[0]

        if len(parts) > 1 and parts[1].startswith("QR="):
            qr_data = parts[1].replace("QR=", "")
        else:
            qr_data = "NONE"

        if qr_data != "NONE":
            last_valid_qr = qr_data
            current_qr = qr_data 

        if human_flag == "HUMAN":
            state = "Human detected"
        elif front_close:
            state = "Object detected"
        else:
            state = "Idle"

        if state != last_state:
            if state == "Human detected":
                print("Agent           : Human")
                print("Physical        : HumanBody")
                print("Process         : Interaction")
                print("QR Data         :", last_valid_qr)

            elif state == "Object detected":
                print("Physical        : Object")
                print("Process         : Motion")
                if current_qr:
                    print("QR Data         :", current_qr)

            else:
                print("Process         : Idle")

            last_state = state

    # ---------- CLEAN DISTANCE PRINT (SYNCED) ----------
    if current_qr and front_dist is not None:
        if last_printed_dist is None or abs(front_dist - last_printed_dist) > DISTANCE_EPS:
            print(f"{current_qr} -> {round(front_dist,2)} m")
            last_printed_dist = front_dist
