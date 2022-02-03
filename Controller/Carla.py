import pygame

from Controller.Controller import Controller


class CARLA(Controller):
    """
    Does Nothing. Lets Carla's Autopilot drive the car around
    """
    def control(self, simulation, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    return True
