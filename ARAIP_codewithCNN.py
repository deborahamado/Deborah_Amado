from controller import Robot

TIME_STEP = 32
MAX_SPEED = 6.28
OBSTACLE_THRESHOLD = 500
ALL_COLOURS = {"red", "green", "blue"}

robot = Robot()

# -----------------------
# Motors
# -----------------------
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# -----------------------
# Distance sensors
# -----------------------
ps = []
for i in range(8):
    sensor = robot.getDevice('ps' + str(i))
    sensor.enable(TIME_STEP)
    ps.append(sensor)

# -----------------------
# Camera
# -----------------------
cam = robot.getDevice('camera')
cam.enable(TIME_STEP)

W = cam.getWidth()
H = cam.getHeight()

seen = set()
photo_taken = False

# Photo settings (tweak these if the photo isn't framed well)
MIN_WIDTH = 0.50      # block must fill at least this much of the image width
MAX_WIDTH = 0.85      # above this the robot is too close and backs up
CENTRE_TOLERANCE = 0.15   # how close to the middle the block must be (0 = perfect)
MIN_RED_AREA = 0.02   # at least 2% of the image must be red to count as "red block"


def clamp(v):
    return max(-MAX_SPEED, min(MAX_SPEED, v))


def red_pixel(r, g, b):
    # Red must be bright and clearly stronger than green and blue
    return (r > 100) and (r > g * 2) and (r > b * 2)


# -----------------------
# Main loop
# -----------------------
while robot.step(TIME_STEP) != -1:
    # LAYER 0: WANDER (default, lowest priority)
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED

    # Read sensors (shared by all layers)
    ps_values = [sensor.getValue() for sensor in ps]

    # LAYER 1: OBSTACLE AVOIDANCE (suppresses layer 0)
    if any(value > OBSTACLE_THRESHOLD for value in ps_values):
        left_speed = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED

    # LAYER 2: COLOUR SCANNING
    img = cam.getImage()
    x = W // 2
    y = H // 2

    r = cam.imageGetRed(img, W, x, y)
    g = cam.imageGetGreen(img, W, x, y)
    b = cam.imageGetBlue(img, W, x, y)

    # Scan the whole image for red (works even if a photo covers the centre)
    red_count = 0
    sum_x = 0
    min_x = W
    max_x = -1
    for yy in range(H):
        for xx in range(W):
            pr = cam.imageGetRed(img, W, xx, yy)
            pg = cam.imageGetGreen(img, W, xx, yy)
            pb = cam.imageGetBlue(img, W, xx, yy)
            if red_pixel(pr, pg, pb):
                red_count += 1
                sum_x += xx
                if xx < min_x:
                    min_x = xx
                if xx > max_x:
                    max_x = xx

    red_found = red_count > MIN_RED_AREA * W * H

    if red_found:
        colour = "red"
    elif (b > r + 20) and (b > g + 20):
        colour = "blue"
    elif (g > r + 20) and (g > b + 20):
        colour = "green"
    else:
        colour = None

    if colour is not None and colour not in seen:
        seen.add(colour)
        print("I see " + colour, r, g, b)

        if len(seen) >= 2:
            print("Seen so far: " + ", ".join(sorted(seen)))

    # LAYER 3: APPROACH THE RED BLOCK AND PHOTOGRAPH ITS FACE
    # (overrides wandering and obstacle avoidance until the photo is taken)
    if red_found and not photo_taken:
        centre_x = (min_x + max_x) / 2
        error = (centre_x - W / 2) / (W / 2)      # -1 (far left) to +1 (far right)
        width_fraction = (max_x - min_x + 1) / W
        fully_in_frame = (min_x > 0) and (max_x < W - 1)

        if width_fraction > MAX_WIDTH:
            # Too close: back up so the whole face fits in the picture
            left_speed = -0.3 * MAX_SPEED
            right_speed = -0.3 * MAX_SPEED
        else:
            # Steer towards the block and drive forwards
            left_speed = clamp(0.4 * MAX_SPEED + 0.3 * MAX_SPEED * error)
            right_speed = clamp(0.4 * MAX_SPEED - 0.3 * MAX_SPEED * error)

        if (MIN_WIDTH < width_fraction <= MAX_WIDTH
                and abs(error) < CENTRE_TOLERANCE
                and fully_in_frame):
            cam.saveImage("red_block.png", 100)
            print("Photo saved: red_block.png")
            photo_taken = True

    # ACTIVATION (final motor commands)
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

    # Stop only when all colours are seen AND the red photo has been taken
    if seen == ALL_COLOURS and photo_taken:
        print("All three blocks found. Search complete.")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break