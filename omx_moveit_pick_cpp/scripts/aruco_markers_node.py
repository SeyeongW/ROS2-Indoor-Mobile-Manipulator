#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers # [수정] 올바른 메시지 임포트
import cv2
import numpy as np
import tf2_ros

def rotation_matrix_to_quaternion(R: np.ndarray):
    """3x3 회전 행렬을 쿼터니언(x, y, z, w)으로 변환"""
    tr = float(np.trace(R))
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return [float(qx), float(qy), float(qz), float(qw)]

class ArucoMarkerNode(Node):
    def __init__(self):
        super().__init__("aruco_markers_node")

        if hasattr(cv2, "ocl"):
            cv2.ocl.setUseOpenCL(False)
        cv2.setNumThreads(1)

        # 파라미터 설정
        self.declare_parameter("dictionary", "DICT_5X5_100")
        self.declare_parameter("marker_size", 0.06)
        self.declare_parameter("image_topic", "/camera/depth_camera/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth_camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth_camera/camera_info")
        self.declare_parameter("annotated_image_topic", "/aruco/markers/image")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("enable_pose", True)

        dict_str = str(self.get_parameter("dictionary").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        image_topic = str(self.get_parameter("image_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        debug_topic = str(self.get_parameter("annotated_image_topic").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.enable_pose = bool(self.get_parameter("enable_pose").value)

        # QoS 설정
        self.qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ArUco 설정
        self.aruco_available = hasattr(cv2, "aruco")
        if not self.aruco_available:
            self.get_logger().error("OpenCV ArUco module unavailable.")
            self.aruco_dict = None
            self.aruco_params = None
        else:
            self.aruco_dict = self.get_aruco_dictionary(dict_str)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_depth_image = None 

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # [수정] 퍼블리셔 메시지 타입 변경 (MarkerArray -> ArucoMarkers)
        self.marker_pub = self.create_publisher(ArucoMarkers, "/aruco/markers", 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)

        # 서브스크립션
        self.create_subscription(CameraInfo, info_topic, self.info_callback, self.qos_reliable)
        self.create_subscription(Image, image_topic, self.image_callback, self.qos_reliable)
        self.create_subscription(Image, depth_topic, self.depth_callback, self.qos_reliable)

        self.get_logger().info(f"ArUco Node Started. Topics: {image_topic}, {depth_topic}")

    def get_aruco_dictionary(self, dict_name: str):
        aruco_dict = {
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        }
        return cv2.aruco.getPredefinedDictionary(aruco_dict.get(dict_name, cv2.aruco.DICT_5X5_100))

    def info_callback(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((1, -1)) if len(msg.d) > 0 else np.zeros((1, 5))
        self.camera_matrix = np.ascontiguousarray(camera_matrix)
        self.dist_coeffs = np.ascontiguousarray(dist_coeffs)
        self.get_logger().info("Camera Info Received.")

    def depth_callback(self, msg: Image):
        try:
            if msg.encoding == "16UC1": # mm -> meters
                scale = 0.001
                dtype = np.uint16
            elif msg.encoding == "32FC1": # meters
                scale = 1.0
                dtype = np.float32
            else:
                return
            
            np_arr = np.frombuffer(msg.data, dtype=dtype)
            if msg.width * msg.height != np_arr.size: return
            depth_img = np_arr.reshape((msg.height, msg.width))
            self.latest_depth_image = depth_img.astype(np.float32) * scale
        except Exception:
            pass

    def get_depth_at_center(self, corners):
        if self.latest_depth_image is None: return -1.0
        c = corners[0]
        cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
        h, w = self.latest_depth_image.shape
        if 0 <= cx < w and 0 <= cy < h:
            val = self.latest_depth_image[cy, cx]
            if np.isfinite(val) and val > 0.0: return float(val)
        return -1.0

    def _to_gray(self, msg: Image):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if "8" in msg.encoding and "c" not in msg.encoding.lower(): 
                 channels = 3
                 if buf.size == msg.width * msg.height * 3:
                     img = buf.reshape((msg.height, msg.width, 3))
                     if "bgr" in msg.encoding.lower():
                         img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                     return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        except Exception:
            pass
        return None

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or not self.aruco_available: return

        gray = self._to_gray(msg)
        if gray is None: return

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0: return

        # [수정] ArucoMarkers 메시지 생성 (MarkerArray가 아님!)
        aruco_markers_msg = ArucoMarkers()
        aruco_markers_msg.header = msg.header
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

        for i in range(len(ids)):
            # 1. ID 추가
            current_id = int(ids[i][0])
            aruco_markers_msg.marker_ids.append(current_id)

            # 2. Pose 생성 및 계산
            pose = Pose()
            
            # Depth 보정
            measured_z = self.get_depth_at_center(corners[i])
            if measured_z > 0.0: tvecs[i][0][2] = measured_z

            pose.position.x = float(tvecs[i][0][0])
            pose.position.y = float(tvecs[i][0][1])
            pose.position.z = float(tvecs[i][0][2])

            rmat = cv2.Rodrigues(rvecs[i])[0]
            qx, qy, qz, qw = rotation_matrix_to_quaternion(rmat)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            
            # 3. Pose 리스트에 추가
            aruco_markers_msg.poses.append(pose)

            # 4. TF 브로드캐스팅 (필수!)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = msg.header.frame_id
            t.child_frame_id = f"aruco_marker_{current_id}"

            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation

            self.tf_broadcaster.sendTransform(t)

        # 퍼블리시
        self.marker_pub.publish(aruco_markers_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()