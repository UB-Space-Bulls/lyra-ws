import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

import pyzed.sl as sl
import cv2
import numpy as np


def find_ar_tag(image):
    """
    Detects AR tags in an image and returns their corner coordinates.

    Args:
        image: Input image (numpy array) or path to image file

    Returns:
        list: List of detected AR tags, where each tag is a dictionary containing:
            - 'corners': numpy array of 4 corner points (shape: 4x2)
            - 'id': tag ID (if using ArUco dictionary)
    """
    # Load image if path is provided
    if isinstance(image, str):
        image = cv2.imread(image)
        if image is None:
            raise ValueError(f"Could not load image from {image}")

    # Convert to grayscale
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    # Initialize ArUco detector with 4x4 dictionary (older OpenCV API)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect markers
    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )

    results = []
    if ids is not None:
        for i, corner in enumerate(corners):
            # corner is shape (1, 4, 2), flatten to (4, 2)
            tag_info = {"corners": corner.reshape(4, 2), "id": int(ids[i][0])}
            results.append(tag_info)

    return results


class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        self.get_logger().info('Vision node started')

        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720 
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Failed to open ZED")

        self.runtime = sl.RuntimeParameters()

        self.ar_pos = self.create_publisher(PoseArray, '/ar_pos', 10)

        self.timer = self.create_timer(0.05, self.fetch_vision)

    def fetch_vision(self):
        image_left = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()

        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return
            
        self.zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU)
        self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)

        image_np = image_left.get_data()

        tag_img_coords = find_ar_tag(image_np) # this is a LIST!

        local_world_coords = list()
        for coord in tag_img_coords:
            err, point_cloud_value = point_cloud.get_value(*coord)
            if err == sl.SUCCESS:
               local_world_coords.append(point_cloud_value[:3]) # 3D point in Camera Frame

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera_frame'
        
        for coord in local_world_coords:
            pose = Pose()
            pose.position.x = float(coord[0])
            pose.position.y = float(coord[1])
            pose.position.z = float(coord[2])
            pose_array.poses.append(pose)
        
        self.ar_pos.publish(pose_array)

    def close(self):
        self.zed.close()


def main(args=None):
    rclpy.init(args=args)
    node = Vision()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
