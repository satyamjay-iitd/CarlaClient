import carla
import pygame
from pygame.locals import KMOD_CTRL
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_LEFT
from pygame.locals import K_RIGHT
from pygame.locals import K_SPACE
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_d
from pygame.locals import K_q
from pygame.locals import K_s
from pygame.locals import K_w

from Controller.Controller import Controller


class Manual(Controller):
    """
    Let the vehicle controlled by keyboard
    """
    def __init__(self):
        self.__steer_cache = 0.0
        self._control = carla.VehicleControl()
        self._lights = carla.VehicleLightState.NONE

    def control(self, simulation, clock):
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1

        self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
        self._control.reverse = self._control.gear < 0
        # Set automatic control-related vehicle lights
        if self._control.brake:
            current_lights |= carla.VehicleLightState.Brake
        else:  # Remove the Brake flag
            current_lights &= ~carla.VehicleLightState.Brake
        if self._control.reverse:
            current_lights |= carla.VehicleLightState.Reverse
        else:  # Remove the Reverse flag
            current_lights &= ~carla.VehicleLightState.Reverse
        if current_lights != self._lights:  # Change the light state only if necessary
            self._lights = current_lights
            simulation.vehicle.set_light_state(carla.VehicleLightState(self._lights))
        simulation.vehicle.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1.00)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
