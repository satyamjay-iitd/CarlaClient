import numpy as np
import carla


def get_actor_blueprints(world, _filter, generation=None):
    bps = world.get_blueprint_library().filter(_filter)

    if generation is None:
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


def get_intrinsic_matrix(camera_bp):
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()
    focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = image_w / 2.0
    K[1, 2] = image_h / 2.0
    return K


# Convert raw image from CARLA to Numpy image
def carla_img_to_np_image(image):
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    image = array[:, :, ::-1]
    return image


# Sensor/Vehicle to World Space
def sensor2world(sensor_transform, points):
    return np.dot(sensor_transform.get_matrix(), points)


# World to Sensor/Vehicle Space
def world2sensor(sensor_transform, points):
    return np.dot(sensor_transform.get_inverse_matrix(), points)


# Sensor/Vehicle to Sensor/Vehicle Space
def sensor2sensor(sensor1_transform, sensor2_transform, points):
    return world2sensor(sensor1_transform,
                        sensor2world(sensor2_transform, points))
