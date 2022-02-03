import random
import copy

import numpy as np
import carla
import pygame

import utils
from ObstacleDetection import ObstacleDetector
from Controller import ControllerEnum, Controller, get_controller
from LaneDetection import LaneDetector
from LaneDetection.camera_geometry import CameraGeometry
from UI import show_ui
import SensorManager
import DisplayManager


spawn_actor_command = carla.command.SpawnActor
set_autopilot_command = carla.command.SetAutopilot
future_actor_command = carla.command.FutureActor


class Simulator:
    def __init__(self, host='127.0.0.1', port=2000, set_autopilot=True, window_w=1024, window_h=1024):
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)

        self.world = self.client.get_world()
        self.original_settings = self.world.get_settings()

        self.traffic_manager = self.client.get_trafficmanager(8000)
        settings = self.world.get_settings()
        self.traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        self.world.apply_settings(settings)

        self.display_manager = DisplayManager.DisplayManager(grid_size=[2, 1], window_size=[window_w, window_h])

        vehicle_bp = self.world.get_blueprint_library().filter('vehicle.audi.tt')[0]
        self.vehicle = self.world.spawn_actor(vehicle_bp, random.choice(self.world.get_map().get_spawn_points()))
        self.vehicle.set_autopilot(set_autopilot)
        self.vehicle_list = [self.vehicle]
        self.pedestrians_and_controllers = []               # list is [controller, actor, controller, actor ...]

        self.timer = SensorManager.CustomTimer()

    def revert_to_original_setting(self):
        self.world.apply_settings(self.original_settings)

    def spawn_vehicle(self, num_vehicles, safe_only=True, car_lights_on=False):
        if num_vehicles == 0:
            return

        print("Trying to spawn {} vehicles".format(num_vehicles))
        blueprints = utils.get_actor_blueprints(self.world, "vehicle.*")

        if safe_only:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]
            blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = self.world.get_map().get_spawn_points()
        num_spawn_points = len(spawn_points)

        if num_vehicles < num_spawn_points:
            random.shuffle(spawn_points)
        elif num_vehicles > num_spawn_points:
            print('requested {} vehicles, but could only find {} spawn points'.format(num_vehicles, num_spawn_points))
            num_vehicles = num_spawn_points

        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= num_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(spawn_actor_command(blueprint, transform)
                         .then(set_autopilot_command(future_actor_command, True, self.traffic_manager.get_port())))

        for response in self.client.apply_batch_sync(batch, True):
            if response.error:
                print(response.error)
            else:
                self.vehicle_list.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if car_lights_on:
            all_vehicle_actors = self.world.get_actors(self.vehicle_list)
            for actor in all_vehicle_actors:
                self.traffic_manager.update_vehicle_lights(actor, True)

        print("Spawned {} vehicles".format(len(self.vehicle_list)))

    def spawn_pedestrians(self, num_pedestrian):
        if num_pedestrian == 0:
            return
        blueprints_walkers = utils.get_actor_blueprints(self.world, "walker.pedestrian.*")
        percentage_pedestrians_running = 0.0  # how many pedestrians will run
        percentage_pedestrians_crossing = 0.0  # how many pedestrians will walk through the road

        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(num_pedestrian):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        walkers_list = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprints_walkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if random.random() > percentage_pedestrians_running:
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(spawn_actor_command(walker_bp, spawn_point))

        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                print(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(spawn_actor_command(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                print(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            self.pedestrians_and_controllers.append(walkers_list[i]["con"])
            self.pedestrians_and_controllers.append(walkers_list[i]["id"])
        all_actors = self.world.get_actors(self.pedestrians_and_controllers)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        self.world.tick()

        # 5. initialize each controller and set target to walk to (list is [controller, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        self.world.set_pedestrians_cross_factor(percentage_pedestrians_crossing)
        for i in range(0, len(self.pedestrians_and_controllers), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))


def add_features(simulator: Simulator, ld_selected: bool, od_selected: bool):
    world = simulator.world
    if ld_selected:
        print("Initializing Lane Detection")
        cg = CameraGeometry()
        ld = LaneDetector(cam_geom=cg)

        print("\t Adding Camera Sensor")
        camera = SensorManager.SensorManager(world, simulator.display_manager, 'RGBCamera',
                                             carla.Transform(carla.Location(x=0.5, z=1.3),
                                                             carla.Rotation(yaw=0, pitch=-5, roll=0)),
                                             simulator.vehicle,
                                             {'fov': '45', 'image_size_x': '1024', 'image_size_y': '512'},
                                             display_pos=[0, 0])
                                             
        def lane_detection_listener(image):
            # image = carla_img_to_np_image(image)
            image.convert(carla.ColorConverter.Raw)
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]

            poly_left, poly_right, left_mask, right_mask = ld.get_fit_and_probs(array)
            # Overlay the detected lane lines on the image for visualization
            overlay = copy.copy(array)
            overlay[left_mask > 0.5, :] = [0, 0, 255]
            overlay[right_mask > 0.5, :] = [255, 0, 0]
            if camera.display_man.render_enabled():
                camera.surface = pygame.surfarray.make_surface(overlay.swapaxes(0, 1))
            camera.tics_processing += 1

        camera.set_sensor_listener(lane_detection_listener)
        print("Camera Sensor Ready")

    if od_selected:
        print("Initializing Obstacle Detection")
        print("\t Adding Lidar Sensor")
        lidar = SensorManager.SensorManager(world, simulator.display_manager, 'SemanticLiDAR',
                                            carla.Transform(carla.Location(x=0, z=2.4)), simulator.vehicle,
                                            {'channels': '64', 'range': '100', 'points_per_second': '100000',
                                             'rotation_frequency': '20'})

        visualization_cam = SensorManager.SensorManager(world, simulator.display_manager, 'RGBCamera',
                                                        carla.Transform(carla.Location(x=0, z=2.4)),
                                                        simulator.vehicle,
                                                        {'image_size_x': '1024', 'image_size_y': '512'},
                                                        display_pos=[1, 0])
        od = ObstacleDetector()
        bboxes = list()

        def obstacle_detection_listener(point_cloud):
            # sensor_transform = point_cloud.transform
            data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))
            points = np.array([data['x'], data['y'], data['z']]).T
            labels = np.array(data['ObjTag'])
            bboxes.clear()
            bboxes.extend(od.get_bboxes(points, labels))
            # vehicle_transform = carla.Transform(sim.vehicle.get_transform())
            # utils.sensor2sensor(sensor_transform,
            #                     vehicle_transform, bboxes)

        lidar.set_sensor_listener(obstacle_detection_listener)

        bb_color = (248, 64, 24)
        calibration_matrix = utils.get_intrinsic_matrix(visualization_cam.get_sensor_bp())

        def obstacle_detection_visualizer(image):
            image = utils.carla_img_to_np_image(image)
            if camera.display_man.render_enabled():
                surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
                surface.set_colorkey((0, 0, 0))
                bboxes_cam_space = []
                for bbox in bboxes:
                    cords_y_minus_z_x = np.concatenate([bbox[1, :], -bbox[2, :], bbox[0, :]])
                    camera_bbox = np.transpose(np.dot(calibration_matrix, cords_y_minus_z_x))
                    camera_bbox = np.concatenate([camera_bbox[:, 0] / camera_bbox[:, 2],
                                                  camera_bbox[:, 1] / camera_bbox[:, 2], camera_bbox[:, 2]], axis=1)
                    bboxes_cam_space.append(camera_bbox)

                for bbox in bboxes_cam_space:
                    points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
                    # draw lines
                    # base
                    pygame.draw.line(surface, bb_color, points[0], points[1])
                    pygame.draw.line(surface, bb_color, points[0], points[1])
                    pygame.draw.line(surface, bb_color, points[1], points[2])
                    pygame.draw.line(surface, bb_color, points[2], points[3])
                    pygame.draw.line(surface, bb_color, points[3], points[0])
                    # top
                    pygame.draw.line(surface, bb_color, points[4], points[5])
                    pygame.draw.line(surface, bb_color, points[5], points[6])
                    pygame.draw.line(surface, bb_color, points[6], points[7])
                    pygame.draw.line(surface, bb_color, points[7], points[4])
                    # base-top
                    pygame.draw.line(surface, bb_color, points[0], points[4])
                    pygame.draw.line(surface, bb_color, points[1], points[5])
                    pygame.draw.line(surface, bb_color, points[2], points[6])
                    pygame.draw.line(surface, bb_color, points[3], points[7])
                visualization_cam.surface = surface
        visualization_cam.set_sensor_listener(obstacle_detection_visualizer)


def run_simulation(simulator, controller: Controller):
    # Simulation loop
    clock = pygame.time.Clock()
    try:
        while True:
            simulator.world.tick()
            clock.tick_busy_loop(60)
            if controller.control(simulator, clock):
                break

            simulator.display_manager.render()

    finally:
        if simulator.display_manager:
            simulator.display_manager.destroy()
        simulator.client.apply_batch([carla.command.DestroyActor(x) for x in simulator.vehicle_list])
        simulator.revert_to_original_setting()


if __name__ == "__main__":
    ld_enabled, od_enabled, selected_controller, vehicles2spawn, ped2spawn = show_ui()
    sim = Simulator(set_autopilot=(selected_controller == ControllerEnum.CARLA.value))
    sim.spawn_vehicle(vehicles2spawn)
    sim.spawn_pedestrians(ped2spawn)
    selected_controller = get_controller(selected_controller)
    add_features(sim, ld_enabled, od_enabled)
    run_simulation(sim, selected_controller)
