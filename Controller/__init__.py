from enum import Enum

from Controller.Controller import Controller
from Controller.Carla import CARLA
from Controller.Manual import Manual


class ControllerEnum(Enum):
    MANUAL = 'Manual'
    CARLA = 'Carla'
    OUR = 'OurAgent'


def get_controller(name: ControllerEnum):
    if name == ControllerEnum.MANUAL.value:
        return Manual()
    elif name == ControllerEnum.CARLA.value:
        return CARLA()
    elif name == ControllerEnum.OUR.value:
        raise NotImplementedError
