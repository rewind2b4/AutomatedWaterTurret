import pygame
import sys
import platform

platform = platform.system()
print(platform)

if platform == 'Linux':
    from io import BytesIO
    from time import sleep
    from picamera import PiCamera

    # Create an in-memory stream
    my_stream = BytesIO()
    camera = PiCamera()
    camera.start_preview()
    # Camera warm-up time
    sleep(2)

pygame.init()
pygame.display.set_caption("Watercannon Turret Control")
screen = pygame.display.set_mode([640, 480])

try:
    while True:
        screen.fill([0, 0, 0])

        if platform == 'Linux':
            camera.capture(frame, 'jpeg')

        frame = pygame.surfarray.make_surface(frame)
        screen.blit(frame, (0, 0))
        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

except:
    exit()

