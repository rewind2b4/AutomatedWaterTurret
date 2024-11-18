# automated_water_turret
Requires https://github.com/FRC4564/Maestro to work


## Configuration file example

GUI: 1                                      ## Turns on or off the video output

HUD: 1                                      ## Turns on or off info overlay on the video

Hold_fire: 1                                ## In manual mode, determines whether the cannon will stay on or shut off after firing

bounding_matrix:                            ## The bounding matrix is the coordinates where the out of bounds areas for the turret are stored

  0:                                        ## The first bounding box, add more bounding boxes in the same format
    0: 100                                  ## The leftmost x coordinate for the bounding box
    1: 300                                  ## The rightmost x coordinate for the bounding box
    2: 100                                  ## The leftmost y coordinate for the bounding box
    3: 400                                  ## The rightmost y coordinate for the bounding box
  1:                                        ## The second bounding box, add more bounding boxes in the same format
    0: 630
    1: 670
    2: 300
    3: 350

camera_init: 1                              ## Forces the camera to fail if 0 (make sure this equals 1)

com_port: COM3                              ## For windows, changes what COM port is used for the Command Port of the Maestro servo controller

continue_anyway: 1                          ## Continues the program even if errors exist if set to 1

contour_area: 50                            ## The minimum area in pixels that a target must take up
detect_min_height: 100                      ## The minimum hieght of a target in pixels
detect_min_width: 100                       ## The minimum width of a target in pixels

firing_interval: 30                         ## Changes how many video frames will pass before the turret fires

flip_image: 1                               ## Flips the video output by 180 degrees

manual_mode: 1                              ## Manual mode is on by default if 1

max_servo_x: 4350                           ## Minimum and maximum servo values, change these within the program.
max_servo_y: 5900
min_servo_x: 6850
min_servo_y: 7500

safety: 0                                   ## safety is on by default if 1

servo_init: 1                               ## Forces the servos to fail if 0 (make sure this equals 1)

servo_tol: 50                               ## Changes the required accuracy for the turret to be aimed correctly

speed: 50                                   ## Changes how far the turret moves per interval in manual mode

speed_interval: 5                           ## Changes the amount of ticks need to pass per interval (Don't change)

tracking_interval: 30                       ## Changes how many video frames will pass before the turret tracks a new target

video_input: 1                              ## Determines which camera the program will use. Starts at 0

x_direction_set: 1                          ## Changes the direction that the x servo will travel relative to the camera

y_direction_set: 1                          ## Changes the direction that the y servo will travel relative to the camera
