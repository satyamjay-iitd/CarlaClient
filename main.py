import numpy as np
import carla
import pygame

from Controller import ControllerEnum, Controller, get_controller
from Simulation import Simulator
from UI import show_ui

import warnings

warnings.simplefilter('ignore', np.RankWarning)


def run_simulation(simulator, controller: Controller):
    # Simulation loop
    clock = pygame.time.Clock()
    try:
        while True:
            simulator.world.tick()
            clock.tick_busy_loop(60)
            if controller.control(simulator, clock):
                break
            simulator.world.tick()
            simulator.display_manager.render()

    finally:
        if simulator.display_manager:
            simulator.display_manager.destroy()
        simulator.client.apply_batch([carla.command.DestroyActor(x) for x in simulator.vehicle_list])
        simulator.revert_to_original_setting()


if __name__ == "__main__":
    ld_enabled, od_enabled, selected_controller, vehicles2spawn, ped2spawn = show_ui()
    sim = Simulator(set_autopilot=(selected_controller == ControllerEnum.CARLA.value),
                    enable_ld=ld_enabled, enable_od=od_enabled)
    sim.spawn_vehicle(vehicles2spawn)
    sim.spawn_pedestrians(ped2spawn)
    selected_controller = get_controller(selected_controller)
    add_features(sim, ld_enabled, od_enabled)
    run_simulation(sim, selected_controller)
