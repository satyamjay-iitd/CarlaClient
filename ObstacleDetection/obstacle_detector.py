import open3d as o3d
import numpy as np

VEHICLE_LABEL = 10
PEDESTRIAN_LABEL = 4


class ObstacleDetector:
    def __init__(self):
        pass

    def get_bboxes(self, points, labels):
        vehicle_points = self.filter_points_by_label(points, labels, VEHICLE_LABEL)
        pedestrian_points = self.filter_points_by_label(points, labels, PEDESTRIAN_LABEL)

        v_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(vehicle_points))
        p_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pedestrian_points))

        vehicle_bboxes = self.__cluster_and_get_bbox(v_pcd)
        pedestrian_bboxes = self.__cluster_and_get_bbox(p_pcd)

        return vehicle_bboxes, pedestrian_bboxes

    @staticmethod
    def filter_points_by_label(points, labels, label):
        return points[np.where(labels == label)]

    @staticmethod
    def __cluster_and_get_bbox(pcd):
        labels = np.array(
            pcd.cluster_dbscan(eps=0.02, min_points=30, print_progress=False))
        max_label = labels.max()
        bboxes = []
        if max_label != -1:
            for i in range(max_label + 1):
                obstacle = pcd.select_by_index(np.where(labels == i)[0])
                obstacle_bbox = np.asarray(obstacle.get_axis_aligned_bounding_box().get_box_points())
                bboxes.append(obstacle_bbox)
        return bboxes
