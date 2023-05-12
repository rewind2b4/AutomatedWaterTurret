import pygame
import maestro
import cv2
import numpy as np
import platform

# ---------------------------- PARAMETERS ------------------------------------------
manual_mode = 0  # enables keyboard control
safety = 0  # disables relay control
GUI = 1  # enables UI, not suitable for SSH
speed = 100  # turret movement speed
HUD = 1  # object detection HUD
video_input = 0  # the video port of the pc
com_port = 'COM3'  # The command port of the maestro controller (Required for windows only)
flip_image = 0    # whether the image needs to be flipped
Hold_fire = 1  # allows fire key to be held on (Manual mode only)

# these can be used to force an error for either camera or servo (Make sure you keep them equalling 1)
servo_init = 1
camera_init = 1

continue_anyway = 1  # disables error checking (for testing only)

# Servo parameters
x_calibration = 1  # calibration multiplier
y_calibration = 1
servo_tol = 50

# Motion detection parameters
tracking_interval = 30
firing_interval = 4

bounding_top = 50
bounding_bottom = 450

detect_min_width = 80
detect_min_height = 80

target_crossref_tol = 1000

# ----------------------------- INITIAL VALUES ----------------------------------
servo_x = 6000  # servo 90degrees
servo_y = 6000  # servo 90degrees
relay1 = 8000  # relay low
relay2 = 8000  # relay low
relay3 = 8000  # relay low

# colour values
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)

# servo initial speed = 0
x_direction = 0
y_direction = 0

# servo target init
target_x = 0
target_y = 0

# Fire control init
fire = 0
fire_complete = 0


# Clock values for servo speed
IMAGE_INTERVAL = 10
last_update = 0

# Motion detection init
target_detected = 0
frame_count = 0

# Servo feedback
servo_x_act = 0
servo_y_act = 0
relay1_act = 0
relay2_act = 0
relay3_act = 0


# ------------------------------------- FUNCTIONS --------------------------------------
def text_display(input_text, var, x_pos, y_pos):  # always format as {x}
    if var != 0:
        formatted_text = font.render(input_text.format(x=var), True, black, white)
    else:
        formatted_text = font.render(input_text, True, black, white)
    text_rect = formatted_text.get_rect()  # create a rectangular object for the text surface object
    text_rect.center = (x_pos // 2, y_pos // 2)  # set the center of the rectangular object.
    screen.blit(formatted_text, text_rect)


def pixeltoservo_x(x):
    # output = (input - input_start)*output_range / input_range + output_start;
    convert_x = ((x - 0) * (8000 / 640) + 2000) * x_calibration
    return convert_x


def pixeltoservo_y(y):
    # output = (input - input_start)*output_range / input_range + output_start;
    convert_y = -((y - 480) * (8000 / 480) + 2000) * y_calibration
    return convert_y


# ---------------------- PLATFORM DESIGNATION ------------------------
platform = platform.system()
print("This program is running on " + platform)

# ------------------------------- CAMERA INIT ----------------------------------
try:
    camera = cv2.VideoCapture(video_input)
    ret, Prev_frame = camera.read()  # read camera input
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

    # ------------------------------- MOTION DETECTION ------------------------------------
        if manual_mode == 0:
            if flip_image == 1:
                frame = cv2.flip(frame, 0)
            # Find the absolute difference between the pixels of the prev_frame and current_frame
            # absdiff() will extract just the pixels of the objects that are moving between the two frames
            frame_diff = cv2.absdiff(frame, Prev_frame)
            motion = 0

            # applying Gray scale by converting the images from color to grayscale,
            # This will reduce noise
            gray = cv2.cvtColor(frame_diff, cv2.COLOR_BGR2GRAY)

            # image smoothing also called blurring is applied using gauisian Blur
            # convert the gray to Gausioan blur to detect motion

            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)[1]

            # fill the gaps by dialiting the image
            # Dilation is applied to binary images.
            dilate = cv2.dilate(thresh, None, iterations=4)

            # Finding contour of moving object
            # Contours can be explained simply as a curve joining all the continuous points (along the boundary),
            # having same color or intensity.
            # For better accuracy, use binary images. So before finding contours, we apply threshold aletrnatives
            (contours, _) = cv2.findContours(dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # loop over the contours
            for cnt in contours:
                (motion_x, motion_y, motion_w, motion_h) = cv2.boundingRect(cnt)
                if cv2.contourArea(cnt) > 700 and (motion_x <= 840) and (motion_y >= bounding_top and motion_y <= bounding_bottom):
                    if motion_w >= detect_min_width & motion_h >= detect_min_height:

                        motion += 1

                        detection = np.array([[motion_x, motion_y]])
                        detection_list = np.append(detection_list, detection, axis=0)

            Prev_frame = frame

            if frame_count % tracking_interval == 0:
                print(detection_list)
                num_rows, num_cols = detection_list.shape
                if num_rows > 0:
                    for i in range(0, num_rows):
                        motion_x = detection_list[i, 0]
                        motion_y = detection_list[i, 1]

                        for j in [a for a in range(0, num_rows) if a != motion_x]:  # iterate through the array but skip over x
                            x_comp = detection_list[j, 0]
                            if abs(x_comp - motion_x) < target_crossref_tol:
                                y_comp = detection_list[j, 1]
                                if abs(j - motion_y) < target_crossref_tol:
                                    target = np.array([motion_x, motion_y])
                                    target_detected = 1
                                    break

                detection_list = np.empty((0, 2), int)
                if target_detected == 1:
                    print("Target", target)

    # ----------------------- MANUAL_CONTROL -----------------------------------
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            if servo_init == 1:
                servo.close()
            pygame.quit()
            exit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_m:
                if manual_mode == 1:
                    manual_mode = 0
                else:
                    manual_mode = 1

            if event.key == pygame.K_q:
                if safety == 1:
                    safety = 0
                else:
                    safety = 1

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
                if event.key == pygame.K_g:
                    print("g has been released\n")
                    relay2 = 8000
                if event.key == pygame.K_h:
                    print("h has been released\n")
                    relay3 = 8000

    if pygame.time.get_ticks() - last_update > IMAGE_INTERVAL:
        servo_x += speed * x_direction * 1
        servo_y += speed * y_direction * 1
        last_update = pygame.time.get_ticks()

    # ---------------------------------- SERVO CONTROL --------------------------------------
    # Formula to convert from pixels to servo
    print('Target_detected: ', target_detected)

    if manual_mode == 0 and target_detected == 1:
        servo_x = pixeltoservo_x(float(target[0]))
        servo_y = pixeltoservo_y(float(target[1]))
        print("servo", servo_x, servo_y)

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
        print(type(servo_x))
        print(type(servo_y))

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
            text_display('F to fire', 0, 1180, 100)
            text_display('G & H control aux relays', 0, 1050, 150)

        else:
            pygame.draw.line(screen, black, (0, bounding_top), (840, bounding_top), 4)
            pygame.draw.line(screen, black, (0, bounding_bottom), (840, bounding_bottom), 4)
            if target_detected == 1:
                print("displaying target @ ", target[0], target[1])
                pygame.draw.line(screen, red, (target[0] - 50, target[1]), (target[0] + 50, target[1]), 4)
                pygame.draw.line(screen, red, (target[0], target[1] - 50), (target[0], target[1] + 50), 4)

        text_display('X_Tar: {x}', servo_x, 100, 50)
        text_display('Y_Tar: {x}', servo_y, 100, 100)
        text_display('X_Pos: {x}', servo_x_act, 110, 150)
        text_display('Y_Pos: {x}', servo_y_act, 110, 200)
        text_display('Q to activate safety', 0, 1090, 200)
        text_display('M to activate manual control', 0, 1020, 250)

        if fire_complete == 1:
            text_display('FIRE!!', 0, 650, 500)

        if safety == 1:
            text_display('Safety On', 0, 1160, 500)

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
