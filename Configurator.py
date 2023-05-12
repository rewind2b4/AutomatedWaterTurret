import pygame
import maestro
import cv2
import numpy as np
import platform
import yaml

# --------------------------- config import ---------------------------------------
with open("config.yml", "r") as ymlfile:
    config = yaml.safe_load(ymlfile)

# ---------------------------- PARAMETERS ------------------------------------------

GUI = config["GUI"]  # enables UI, not suitable for SSH
HUD = config["HUD"]  # object detection HUD
video_input = config["video_input"]  # the video port of the pc
com_port = config["com_port"]  # The command port of the maestro controller (Required for windows only)
flip_image = config["flip_image"]   # whether the image needs to be flipped
Hold_fire = config["Hold_fire"]  # allows fire key to be held on (Manual mode only)

# these can be used to force an error for either camera or servo (Make sure you keep them equalling 1)
servo_init = config["servo_init"]
camera_init = config["camera_init"]
continue_anyway = config["continue_anyway"]  # disables error checking (for testing only)

# Servo parameters
x_calibration = config["x_calibration"]  # calibration multiplier
y_calibration = config["y_calibration"]
servo_tol = config["servo_tol"]
speed = config["speed"]  # turret movement speed
firing_interval = config["firing_interval"]
x_direction_set = config["x_direction_set"]
y_direction_set = config["y_direction_set"]

# Motion detection parameters
bounding_top = config["bounding_top"]
bounding_bottom = config["bounding_bottom"]
bounding_left = config["bounding_left"]
bounding_right = config["bounding_right"]

bounding_matrix = config["bounding_matrix"]
try:
    print(len(bounding_matrix))
    bounding_matrix_valid = 1
except:
    bounding_matrix_valid = 0


tracking_interval = config["tracking_interval"]
detect_min_width = config["detect_min_width"]
detect_min_height = config["detect_min_height"]
target_crossref_tol = config["target_crossref_tol"]
contour_area = config["contour_area"]

# ----------------------------- INITIAL VALUES ----------------------------------
servo_x = config["servo_x"]  # servo 90degrees
servo_y = config["servo_y"]  # servo 90degrees
relay1 = config["relay1"]  # relay low
relay2 = config["relay2"]  # relay low
relay3 = config["relay3"]  # relay low

# colour values
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)

# servo initial speed = 0
x_direction = config["x_direction"]
y_direction = config["y_direction"]

# servo target init
target_x = config["target_x"]
target_y = config["target_y"]

# Fire control init
fire = config["fire"]
fire_complete = config["fire_complete"]


# Clock values for servo speed
IMAGE_INTERVAL = config["IMAGE_INTERVAL"]
last_update = config["last_update"]

# Motion detection init
target_detected = config["target_detected"]
frame_count = config["frame_count"]

# Servo feedback
servo_x_act = config["servo_x_act"]
servo_y_act = config["servo_y_act"]
relay1_act = config["relay1_act"]
relay2_act = config["relay2_act"]
relay3_act = config["relay3_act"]

manual_mode = 1  # enables keyboard control
safety = config["safety"]  # disables relay control


# ------------------------------------- FUNCTIONS --------------------------------------
def text_display(input_text, var, x_pos, y_pos):  # always format as {x}
    if var != 0:
        formatted_text = font.render(input_text.format(x=var), True, black, white)
    else:
        formatted_text = font.render(input_text, True, black, white)
    text_rect = formatted_text.get_rect()  # create a rectangular object for the text surface object
    text_rect.center = (x_pos // 2, y_pos // 2)  # set the center of the rectangular object.
    screen.blit(formatted_text, text_rect)


# https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
def remap(x, oMin, oMax, nMin, nMax):
    # range check
    if oMin == oMax:
        print("Warning: Zero input range")
        return None

    if nMin == nMax:
        print("Warning: Zero output range")
        return None

    # check reversed input range
    reverseInput = False
    oldMin = min(oMin, oMax)
    oldMax = max(oMin, oMax)
    if not oldMin == oMin:
        reverseInput = True

    # check reversed output range
    reverseOutput = False
    newMin = min(nMin, nMax)
    newMax = max(nMin, nMax)
    if not newMin == nMin:
        reverseOutput = True

    portion = (x-oldMin)*(newMax-newMin)/(oldMax-oldMin)
    if reverseInput:
        portion = (oldMax-x)*(newMax-newMin)/(oldMax-oldMin)

    result = portion + newMin
    if reverseOutput:
        result = newMax - portion

    return result


# ---------------------- PLATFORM DESIGNATION ------------------------
platform = platform.system()
print("This program is running on " + platform)

# ------------------------------- CAMERA INIT ----------------------------------
try:
    camera = cv2.VideoCapture(video_input)
    ret, Prev_frame = camera.read()  # read camera input

    camera_width = Prev_frame.shape[1]
    camera_height = Prev_frame.shape[0]

except AttributeError:
    print("Camera Error, ensure USB camera / webcam is connected")
    if continue_anyway == 0:
        exit()
    else:
        camera_init = 0

detection_list = np.empty((0, 2), int)
target = np.empty((0, 2), int)

# ----------------------------- PYGAME INIT ----------------------------------
pygame.init()
clock = pygame.time.Clock()

if GUI == 1:
    pygame.display.set_caption("Watercannon Turret Control")

    if camera_init == 1:
        screen = pygame.display.set_mode([camera_width, camera_height])
    else:
        screen = pygame.display.set_mode([640, 480])

    font = pygame.font.Font('freesansbold.ttf', 16)

# ------------------------------ SERVO INIT ----------------------------------
try:
    if platform == 'Windows':
        servo = maestro.Controller(com_port)
    else:
        servo = maestro.Controller()
except:
    print("Servo Error. Ensure USB Servo Controller is connected")
    if continue_anyway == 0:
        exit()
    else:
        servo_init = 0

if servo_init == 1:
    servo.setTarget(0, servo_x)  # set x servo to move to center position (3000 - 9000 is range)
    servo.setTarget(1, servo_y)  # set y servo to move to center position (3000 - 9000 is range)

    servo.setTarget(2, relay1)  # set relay 1 to move to off (0)
    servo.setTarget(3, relay2)  # set relay 2 to move to off (0)
    servo.setTarget(4, relay3)  # set relay 3 to move to off (0)

    servo_x_act = servo.getPosition(0)  # get the current position of x servo
    servo_y_act = servo.getPosition(1)  # get the current position of y servo

    relay1_act = servo.getPosition(2)  # get the current state of relay 1
    relay2_act = servo.getPosition(3)  # get the current state of relay 2
    relay3_act = servo.getPosition(4)  # get the current state of relay 3


# ---------------------- LOOP -----------------------------------
while True:
    frame_count += 1
    print('')
    print('Frame_count: ', frame_count)

    # --------------------------------- WEBCAM INPUT ------------------------------------
    if camera_init == 1:
        ret, frame = camera.read()  # read camera input

        if flip_image == 1:
            frame = cv2.flip(frame, 0)

    # ----------------------- MANUAL_CONTROL -----------------------------------
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            if servo_init == 1:
                servo.close()
            pygame.quit()
            exit()

        if manual_mode == 1:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    y_direction = 1
                    print('y_direction = 1')
                if event.key == pygame.K_d:
                    x_direction = 1
                    print('x_direction = 1')
                if event.key == pygame.K_s:
                    y_direction = -1
                    print('y_direction = -1')
                if event.key == pygame.K_a:
                    x_direction = -1
                    print('x_direction = -1')

                if event.key == pygame.K_f:
                    fire = 1
                    print("f has been pressed")
                if event.key == pygame.K_g:
                    relay2 = 0
                    print("g has been pressed")
                if event.key == pygame.K_h:
                    relay3 = 0
                    print("h has been pressed")

            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_w:
                    y_direction = 0
                    print('y_direction = 0')
                if event.key == pygame.K_d:
                    x_direction = 0
                    print('x_direction = 0')
                if event.key == pygame.K_s:
                    y_direction = 0
                    print('y_direction = 0')
                if event.key == pygame.K_a:
                    x_direction = 0
                    print('x_direction = 0')

                if event.key == pygame.K_f:
                    print("f has been released\n")
                    fire = 0
                if event.key == pygame.K_1:
                    min_servo_x = servo_x
                    min_servo_y = servo_y

                if event.key == pygame.K_2:
                    max_servo_x = servo_x
                    max_servo_y = servo_y

    if pygame.time.get_ticks() - last_update > IMAGE_INTERVAL:
        servo_x += speed * x_direction * x_direction_set
        servo_y += speed * y_direction * y_direction_set
        last_update = pygame.time.get_ticks()

    # ---------------------------------- SERVO CONTROL --------------------------------------
    # Formula to convert from pixels to servo
    print('Target_detected: ', target_detected)

    # X and Y max / min
    if servo_x > 10000:
        servo_x = 10000

    elif servo_x < 2000:
        servo_x = 2000

    if servo_y > 10000:
        servo_y = 10000

    elif servo_y < 2000:
        servo_y = 2000

    if servo_init == 1:
        servo_x = int(servo_x)
        servo_y = int(servo_y)

        print("servo", servo_x, servo_y)

        servo.setTarget(0, servo_x)  # set x servo to move to center position (3000 - 9000 is range)
        servo.setTarget(1, servo_y)  # set y servo to move to center position (3000 - 9000 is range)

        servo_x_act = servo.getPosition(0)  # get the current position of x servo
        servo_y_act = servo.getPosition(1)  # get the current position of y servo

    # ------------------------------ FIRE CONTROL ----------------------------
    if servo_init == 1:
        if manual_mode == 0 and target_detected == 1:
            if abs(servo_x - servo_x_act) < servo_tol:
                fire = 1

        if fire == 1:
            if frame_count % firing_interval == 0:
                fire_complete = 1
                if safety == 0:
                    relay1 = 0
        else:
            relay1 = 8000

        servo.setTarget(2, relay1)  # set relay 1 to move to off (0)
        servo.setTarget(3, relay2)  # set relay 2 to move to off (0)
        servo.setTarget(4, relay3)  # set relay 3 to move to off (0)

        relay1_act = servo.getPosition(2)  # get the current state of relay 1
        relay2_act = servo.getPosition(3)  # get the current state of relay 2
        relay3_act = servo.getPosition(4)  # get the current state of relay 3

    else:
        if manual_mode == 0 and target_detected == 1:
            print('fire complete')
            fire_complete = 1

        elif manual_mode == 1:
            if fire == 1:
                fire_complete = 1
                print('fire complete')

    # -------------------------------------- SCREEN UPDATE --------------------------------------------
    if GUI == 1:
        screen.fill([0, 0, 0])  # fill screen with black pixels to clear

        if camera_init == 1:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # convert to the correct colour space
            frame = np.rot90(frame)  # rotate frame 90 degrees to ensure
            image = pygame.surfarray.make_surface(frame)  # convert image to pygame surface
        else:
            image = pygame.Surface([640, 480])
        screen.blit(image, (0, 0))  # display surface

        if manual_mode == 1:
            text_display('W A S D to position', 0, 1090, 50)
            text_display('F to activate laser pointer', 0, 1040, 100)

        text_display('X_Tar: {x}', servo_x, 100, 50)
        text_display('Y_Tar: {x}', servo_y, 100, 100)
        text_display('X_Pos: {x}', servo_x_act, 110, 150)
        text_display('Y_Pos: {x}', servo_y_act, 110, 200)

        if fire_complete == 1:
            text_display('FIRE!!', 0, 650, 500)

        pygame.display.update()  # update screen

    if fire_complete == 1:
        target_detected = 0
        target = np.empty((0, 2), int)
        if manual_mode == 1:
            if Hold_fire == 1:
                fire = 1
            else:
                fire = 0
        else:
            fire = 0

    fire_complete = 0
