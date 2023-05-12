import pygame
import maestro
import cv2
import numpy as np
import platform
import yaml


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


# --------------------------- config import ---------------------------------------
with open("config.yml", "r") as ymlfile:
    config = yaml.safe_load(ymlfile)

# ---------------------------- PARAMETER DUMP ------------------------------------------

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
servo_tol = config["servo_tol"]
speed = config["speed"]  # turret movement speed
firing_interval = config["firing_interval"]
x_direction_set = config["x_direction_set"]
y_direction_set = config["y_direction_set"]

min_servo_x = int(config["min_servo_x"])
min_servo_y = int(config["min_servo_y"])
max_servo_x = int(config["max_servo_x"])
max_servo_y = int(config["max_servo_y"])

if min_servo_x == max_servo_x:
    min_servo_x = max_servo_x + 100

if min_servo_y == max_servo_y:
    min_servo_y = max_servo_y + 100

# Motion detection parameters
try:
    bounding_matrix = config["bounding_matrix"]
    print(len(bounding_matrix))
    for i in range(0, len(bounding_matrix)):
        if bounding_matrix[i][0]:
            bounding_matrix_valid = 1
        else:
            bounding_matrix_valid = 0
            break
        if bounding_matrix[i][1]:
            bounding_matrix_valid = 1
        else:
            bounding_matrix_valid = 0
            break
        if bounding_matrix[i][2]:
            bounding_matrix_valid = 1
        else:
            bounding_matrix_valid = 0
            break
        if bounding_matrix[i][3]:
            bounding_matrix_valid = 1
        else:
            bounding_matrix_valid = 0
            break

except:
    bounding_matrix_valid = 0

tracking_interval = config["tracking_interval"]
detect_min_width = config["detect_min_width"]
detect_min_height = config["detect_min_height"]
contour_area = config["contour_area"]
bounding_top = 0
bounding_bottom = 800


# ----------------------------- INITIAL VALUES ----------------------------------
servo_x = 6000  # servo 90degrees
servo_y = 6000  # servo 90degrees
relay1 = 8000
relay2 = 8000
relay3 = 8000

x_direction = 0
y_direction = 0

# colour values
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)


# servo target init
target_x = 0
target_y = 0

# Fire control init
fire = 0
fire_complete = 0


# Clock values for servo speed
speed_interval = config["speed_interval"]
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

manual_mode = config["manual_mode"]  # enables keyboard control
safety = config["safety"]  # disables relay control

min_calib_set = 0
max_calib_set = 0
config_write_set = 0

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

    # ------------------------------- MOTION DETECTION ------------------------------------
        if manual_mode == 0:
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
                motion_x = remap(motion_x, 0, 640, 640, 0)  # x is reversed if the image is flipped
                # motion_y = remap(motion_y, 0, 480, 480, 0)  # x is reversed if the image is flipped

                if cv2.contourArea(cnt) > contour_area:
                    target_out_bounds = 0

                    if bounding_matrix_valid == 1:
                        for i in range(0, len(bounding_matrix)):
                            if bounding_matrix[i][0] <= motion_x <= bounding_matrix[i][1] and bounding_matrix[i][2] <= motion_y <= bounding_matrix[i][3]:
                                target_out_bounds = 1

                        if target_out_bounds == 0:

                            if motion_w >= detect_min_width and motion_h >= detect_min_height:

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

                        for j in [a for a in range(0, num_rows) if a != motion_x]:  # iterate the array but skip over x
                            x_comp = detection_list[j, 0]
                            y_comp = detection_list[j, 1]
                            target = np.array([motion_x, motion_y])
                            target_detected = 1
                            break

                detection_list = np.empty((0, 2), int)
                if target_detected == 1:
                    print("Target", target)

    # ----------------------- MANUAL_CONTROL -----------------------------------
    min_calib_set = 0
    max_calib_set = 0
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

                if event.key == pygame.K_1:
                    if servo_init == 1:
                        min_calib_set = 1
                        min_servo_x = servo_x
                        min_servo_y = servo_y

                if event.key == pygame.K_2:
                    if servo_init == 1:
                        max_calib_set = 1
                        max_servo_x = servo_x
                        max_servo_y = servo_y

                if event.key == pygame.K_0:
                    config_write = {'GUI': GUI, 'HUD': HUD, 'video_input': video_input, 'com_port': com_port,
                                    'flip_image': flip_image, 'Hold_fire': Hold_fire, 'servo_init': servo_init,
                                    'camera_init': camera_init, 'continue_anyway': continue_anyway,
                                    'servo_tol': servo_tol, 'speed': speed, 'firing_interval': firing_interval,
                                    'x_direction_set': x_direction_set, 'y_direction_set': y_direction_set,
                                    'min_servo_x': min_servo_x, 'min_servo_y': min_servo_y, 'max_servo_x': max_servo_x,
                                    'max_servo_y': max_servo_y, 'bounding_matrix': bounding_matrix,
                                    'tracking_interval': tracking_interval, 'detect_min_width': detect_min_width,
                                    'detect_min_height': detect_min_height, 'contour_area': contour_area,
                                    'speed_interval': speed_interval, 'manual_mode': 0, 'safety': safety}
                    config_write_set = 1

                    with open("config.yml", "w") as ymlfile:
                        config_written = yaml.dump(config_write, ymlfile)

    if pygame.time.get_ticks() - last_update > speed_interval:
        print(x_direction_set)
        print(y_direction_set)
        print(x_direction)
        print(y_direction)
        servo_x += speed * x_direction * x_direction_set
        servo_y += speed * y_direction * y_direction_set
        last_update = pygame.time.get_ticks()

    # ---------------------------------- SERVO CONTROL --------------------------------------
    # Formula to convert from pixels to servo
    print('Target_detected: ', target_detected)

    if manual_mode == 0 and target_detected == 1:
        if x_direction_set == 1:
            servo_x = remap(float(target[0]), 0, int(camera_width), max_servo_x, min_servo_x)
        else:
            servo_x = remap(float(target[0]), 0, int(camera_width), min_servo_x, max_servo_x)

        if y_direction_set == 1:
            servo_y = remap(float(target[1]), 0, int(camera_height), max_servo_y, min_servo_y)
        else:
            servo_y = remap(float(target[1]), 0, int(camera_height), min_servo_y, max_servo_y)
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

        if servo_init == 0:
            text_display('Servo controller not connected, please restart', 0, 885, 600)

        if camera_init == 0:
            text_display('Camera not connected, please restart', 0, 950, 550)
        if manual_mode == 0:
            if bounding_matrix_valid == 0:
                text_display('No bounding boxes', 0, 1000, 900)

        if manual_mode == 1:
            text_display('W A S D to position', 0, 1090, 150)
            text_display('F to fire', 0, 1180, 200)
            text_display('G & H control aux relays', 0, 1050, 250)
            text_display('0 to save settings', 0, 170, 900)
            if servo_init == 1:
                text_display('1 to set top left calibration', 0, 1040, 300)
                text_display('2 to set bottom right calibration', 0, 1000, 350)
            if min_calib_set == 1:
                text_display('Min calibration set', 0, 150, 400)
                min_calib_set = 0
            if max_calib_set == 1:
                text_display('Max calibration set', 0, 150, 400)
                max_calib_set = 0
            if camera_init == 1:
                text_display('camera height: {x}', camera_height, 1050, 850)
                text_display('camera width: {x}', camera_width, 1050, 900)
            if config_write_set == 1:
                text_display('Settings saved', 0, 145, 800)
                config_write_set = 0


        else:
            if bounding_matrix_valid == 1:
                for i in range(0, len(bounding_matrix)):
                    pygame.draw.line(screen, black, (bounding_matrix[i][0], bounding_matrix[i][2]), (bounding_matrix[i][1], bounding_matrix[i][2]), 4)
                    pygame.draw.line(screen, black, (bounding_matrix[i][0], bounding_matrix[i][3]), (bounding_matrix[i][1], bounding_matrix[i][3]), 4)
                    pygame.draw.line(screen, black, (bounding_matrix[i][0], bounding_matrix[i][2]), (bounding_matrix[i][0], bounding_matrix[i][3]), 4)
                    pygame.draw.line(screen, black, (bounding_matrix[i][1], bounding_matrix[i][2]), (bounding_matrix[i][1], bounding_matrix[i][3]), 4)

            if target_detected == 1:
                print("displaying target @ ", target[0], target[1])
                pygame.draw.line(screen, red, (target[0] - 50, target[1]), (target[0] + 50, target[1]), 4)
                pygame.draw.line(screen, red, (target[0], target[1] - 50), (target[0], target[1] + 50), 4)

        text_display('X_Tar: {x}', servo_x, 150, 50)
        text_display('Y_Tar: {x}', servo_y, 150, 100)
        text_display('X_Pos: {x}', servo_x_act, 160, 150)
        text_display('Y_Pos: {x}', servo_y_act, 160, 200)
        text_display('Q to activate safety', 0, 1090, 50)
        text_display('M to activate manual control', 0, 1020, 100)



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
