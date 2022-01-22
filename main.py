import random
from pathlib import Path
import copy

import numpy as np
import carla
import pygame

from LaneDetection import LaneDetector
from LaneDetection.camera_geometry import CameraGeometry
from UI import MainWindow
from utils import get_intrinsic_matrix, carla_img_to_np_image
import SensorManager
import DisplayManager


class Simulator:
    def __init__(self, host='127.0.0.1', port=2000, set_autopilot=False, window_w=1280, window_h=720):
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)

        self.world = self.client.get_world()
        self.settings = self.world.get_settings()

        self.display_manager = DisplayManager.DisplayManager(grid_size=[1, 1], window_size=[window_w, window_h])

        vehicle_bp = self.world.get_blueprint_library().filter('vehicle.audi.tt')[0]
        self.vehicle = self.world.spawn_actor(vehicle_bp, random.choice(self.world.get_map().get_spawn_points()))
        self.vehicle.set_autopilot(set_autopilot)
        self.vehicle_list = [self.vehicle]

        self.timer = SensorManager.CustomTimer()


def init_simulator():
    return Simulator()


def add_features(sim: Simulator, ld_selected: bool, od_selected: bool):
    world = sim.world
    if ld_selected:
        print("Initializing Lane Detection")
        cg = CameraGeometry()
        ld = LaneDetector(model_path=Path(r"D:\IITD classes\Semester 4\CarlaClient\LaneDetection\fastai_model.pth").absolute(), cam_geom=cg)

        print("\t Adding Camera Sensor")
        camera = SensorManager.SensorManager(world, sim.display_manager, 'RGBCamera',
                                             carla.Transform(carla.Location(x=0.5, z=1.3),
                                                             carla.Rotation(yaw=0, pitch=-5, roll=0)),
                                             sim.vehicle, {'fov': '45',  'image_size_x': '1024', 'image_size_y': '512'},
                                             display_pos=[0, 0])
                                             
        def lane_detection_listener(image):
            image = carla_img_to_np_image(image)
            poly_left, poly_right, left_mask, right_mask = ld.get_fit_and_probs(image)

            # Overlay the detected lane lines on the image for visualization
            overlay = copy.copy(image)
            overlay[left_mask > 0.5, :] = [0, 0, 255]
            overlay[right_mask > 0.5, :] = [255, 0, 0]

            if camera.display_man.render_enabled():
                camera.surface = pygame.surfarray.make_surface(overlay.swapaxes(0, 1))
            camera.tics_processing += 1
        
        camera.set_sensor_listener(lane_detection_listener)
        K = get_intrinsic_matrix(camera.get_sensor_bp())
        print(K)
        print("Camera Sensor Ready")

    if od_selected:
        print("Initializing Obstacle Detection")


def run_simulation(sim):
    # Simulation loop
    call_exit = False
    time_init_sim = sim.timer.time()
    while True:
        sim.world.tick()

        # Render received data
        sim.display_manager.render()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                call_exit = True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    call_exit = True
                    break

        if call_exit:
            break

    if sim.display_manager:
        sim.display_manager.destroy()

    sim.client.apply_batch([carla.command.DestroyActor(x) for x in sim.vehicle_list])
    # sim.world.apply_settings(original_settings)


if __name__ == "__main__":
    ld_enabled, od_enabled = MainWindow.show_ui()
    sim = init_simulator()
    add_features(sim, ld_enabled, od_enabled)
    run_simulation(sim)
