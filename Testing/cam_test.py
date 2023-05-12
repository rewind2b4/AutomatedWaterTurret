import pygame
import cv2
import numpy as np
import sys
import platform

platform = platform.system()
print(platform)

camera = cv2.VideoCapture(0)

pygame.init()
pygame.display.set_caption("Watercannon Turret Control")
screen = pygame.display.set_mode([640, 480])

try:
    while True:
        screen.fill([0, 0, 0])


        ret, frame = camera.read()

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)


        frame = pygame.surfarray.make_surface(frame)
        screen.blit(frame, (0, 0))
        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

except:
    exit()

