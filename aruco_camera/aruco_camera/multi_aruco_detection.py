import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
 
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseArray, Pose
from pick_n_place_interfaces.msg import ArucoMarkers, ArucoMarkersArray
import tf2_ros
from pick_n_place_interfaces.srv import TransformService, RemoveAruco, Reset
import tf2_geometry_msgs
from rclpy.time import Time
 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
 
class ArucoNode(Node):
  def __init__(self):
    super().__init__('aruco_node')

    # Declare parameters
    self.declare_parameter("aruco_dictionary_name", "DICT_6X6_250")
    self.declare_parameter("aruco_marker_side_length", 0.021)
    self.declare_parameter("camera_calibration_parameters_filename", "/home/shu/pick_n_place/aruco_camera/aruco_camera/calibration_data.npz")
    self.declare_parameter("image_topic", "/camera/color/image_raw")
    self.declare_parameter("aruco_marker_name", "aruco_marker")
    self.camera_calibration_parameters_filename = '/home/shu/pick_n_place/aruco_camera/aruco_camera/calibration_data.npz'
     
    self.aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
    self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value
 
    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(self.aruco_dictionary_name, None) is None:
      self.get_logger().info("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))

  # Load the camera parameters from the saved NumPy archive
    with np.load(self.camera_calibration_parameters_filename) as data:
        self.mtx = data['mtx']
        self.dst = data['dist']
     
    # Load the ArUco dictionary
    self.get_logger().info("[INFO] detecting '{}' markers...".format(self.aruco_dictionary_name))
    self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    self.this_aruco_parameters = cv2.aruco.DetectorParameters()
    self.aruco_detector = cv2.aruco.ArucoDetector(self.this_aruco_dictionary, detectorParams=self.this_aruco_parameters)

    try:
        dictionary_id = cv2.aruco.__getattribute__(self.aruco_dictionary_name )
        if type(dictionary_id) != type(cv2.aruco.DICT_6X6_250):
            raise AttributeError
    except AttributeError:
        self.get_logger().error(
            "bad aruco_dictionary_id: {}".format(self.aruco_dictionary_name )
        )
        options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
        self.get_logger().error("valid options: {}".format(options))

    # Set up publishers
    self.markers_pub = self.create_publisher(ArucoMarkersArray, "aruco_markers", 10)


    self.aruco_array = ArucoMarkersArray()
    self.aruco_array.aruco_array = []
    self.transformed_aruco_array = ArucoMarkersArray()
    self.transformed_aruco_array.aruco_array = []
    self.done_aruco = []

    self.subscription = self.create_subscription(Image, self.image_topic, self.listener_callback, 10)

    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
    self.srv = self.create_service(TransformService, 'get_transform', self.handle_transform_request)
    self.get_logger().info("Transform service is ready")
    self.remove_srv = self.create_service(RemoveAruco, 'remove_aruco', self.handle_remove_request)
    self.reset_srv = self.create_service(Reset, 'reset_aruco', self.reset)


    self.tfbroadcaster = TransformBroadcaster(self)
    self.bridge = CvBridge()

################################################# request client #################################################

    self.client = self.create_client(TransformService, 'get_transform')
    while not self.client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Waiting for transform service...')

    self.subscription = self.create_subscription(
        ArucoMarkersArray,
        'aruco_markers',
        self.aruco_pose_callback,
        10
    )

    self.latest_array = None
    self.publisher = self.create_publisher(ArucoMarkersArray, 'transformed_aruco_pose', 10)
    self.timer = self.create_timer(1.0, self.request_transform)

  def aruco_pose_callback(self, msg: ArucoMarkersArray):
      if msg.aruco_array:
          self.latest_array = msg.aruco_array
      else:
          self.latest_array = None  # Clear stale data if no markers are detected


  def request_transform(self):
      if self.latest_array is None:
          self.get_logger().warn("No Aruco marker pose received yet.")
          return

      request = TransformService.Request()
      request.aruco_array = self.latest_array  # This matches the .srv file

      future = self.client.call_async(request)
      future.add_done_callback(self.transform_callback)

  def transform_callback(self, future):
    response = future.result()
    self.transformed_aruco_array = ArucoMarkersArray()
    self.transformed_aruco_array.aruco_array = response.transformed_aruco_array
    self.publisher.publish(self.transformed_aruco_array)
        

################################################# request client #################################################

  def listener_callback(self, data):

    self.get_logger().info('Receiving video frame')
    current_frame = self.bridge.imgmsg_to_cv2(data)
    corners, marker_ids, rejected= self.aruco_detector.detectMarkers(current_frame)
 
    if marker_ids is not None:
      cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        self.aruco_marker_side_length,
        self.mtx,
        self.dst)
      
      current_marker_ids = marker_ids.tolist() if marker_ids is not None else []
      markers_to_remove = []

      for existing_aruco in self.aruco_array.aruco_array:
          if existing_aruco.marker_ids not in current_marker_ids:
              markers_to_remove.append(existing_aruco)

      for marker in markers_to_remove:
          self.aruco_array.aruco_array.remove(marker)

      for i, marker_id in enumerate(marker_ids):  
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = self.aruco_marker_name
       
        # Store the translation (i.e. position) information
        t.transform.translation.x = tvecs[i][0][0]
        t.transform.translation.y = tvecs[i][0][1]
        t.transform.translation.z = tvecs[i][0][2]
 
        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()   
         
        # Quaternion format     
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 

        self.tfbroadcaster.sendTransform(t) 

        if marker_id not in [existing_aruco.marker_ids for existing_aruco in self.aruco_array.aruco_array]:
          aruco= ArucoMarkers()
          aruco.marker_ids = int(marker_id)

          aruco.poses.position.x = t.transform.translation.x
          aruco.poses.position.y = t.transform.translation.y
          aruco.poses.position.z = t.transform.translation.z

          aruco.poses.orientation.x = t.transform.rotation.x
          aruco.poses.orientation.y = t.transform.rotation.y
          aruco.poses.orientation.z = t.transform.rotation.z
          aruco.poses.orientation.w = t.transform.rotation.w

          self.aruco_array.aruco_array.append(aruco)

          self.aruco_array.aruco_array.sort(key=lambda aruco: aruco.marker_ids)
          self.get_logger().info(f"New aruco marker detected! Arcuo_id{marker_id}")
          self.markers_pub.publish(self.aruco_array)

        # Draw the axes on the marker
        cv2.drawFrameAxes(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)    
        self.draw_rectangle(i,rvecs,tvecs,current_frame)
            
    else:
        self.get_logger().warn("No Aruco detected.")
        self.aruco_array.aruco_array = []
        self.markers_pub.publish(self.aruco_array)
        self.latest_array = None

            
    # Display image
    cv2.imshow("camera", current_frame)
    cv2.waitKey(1)

  def draw_rectangle(self,i,rvecs,tvecs,current_frame):
          #----------------------------------------------draw rectangle-------------------------------#
    # Ensure rvecs[i] and tvecs[i] are converted to the correct type
    rvec = np.array(rvecs[i], dtype=np.float64)
    tvec = np.array(tvecs[i], dtype=np.float64)
    # Dimensions of the rectangle in meters (convert mm to meters)
    height = 23 / 1000  # H: 159.9 mm -> meters
    width = 117 / 1000   # W: 76.7 mm -> meters
    depth = 8 / 1000   # D: 8.25 mm -> meters (optional if needed for 3D display)

    # Define the 3D box corners relative to the center
    # Bottom face (4 corners) and top face (4 corners)
    # Adjust the 3D box to align the top surface with the marker
    # Bottom face is now at z = -depth and top face is at z = 0
    box_3d = np.array([
        # Bottom face
        [-width / 2, -height / 2, -depth],  # Bottom-left-front
        [ width / 2, -height / 2, -depth],  # Bottom-right-front
        [ width / 2,  height / 2, -depth],  # Top-right-front
        [-width / 2,  height / 2, -depth],  # Top-left-front

        # Top face
        [-width / 2, -height / 2, 0],       # Bottom-left-back
        [ width / 2, -height / 2, 0],       # Bottom-right-back
        [ width / 2,  height / 2, 0],       # Top-right-back
        [-width / 2,  height / 2, 0]        # Top-left-back
    ], dtype=np.float32)


    # Project the 3D box points into 2D
    box_2d, _ = cv2.projectPoints(box_3d, rvec, tvec, self.mtx, self.dst)

    # Convert to integer for drawing
    box_2d = box_2d.reshape(-1, 2).astype(int)

    # Draw the box: Connect edges
    # Connect bottom face
    for i in range(4):
        pt1 = tuple(box_2d[i])
        pt2 = tuple(box_2d[(i + 1) % 4])  # Loop within bottom face
        cv2.line(current_frame, pt1, pt2, (0, 255, 0), 2)  # Green

    # Connect top face
    for i in range(4, 8):
        pt1 = tuple(box_2d[i])
        pt2 = tuple(box_2d[4 + (i + 1 - 4) % 4])  # Loop within top face
        cv2.line(current_frame, pt1, pt2, (0, 255, 0), 2)  # Green

    # Connect vertical edges
    for i in range(4):
        pt1 = tuple(box_2d[i])       # Bottom face point
        pt2 = tuple(box_2d[i + 4])   # Corresponding top face point
        cv2.line(current_frame, pt1, pt2, (0, 255, 0), 2)  # Green
   
  def handle_transform_request(self, request: TransformService.Request, response: TransformService.Response):
    response.transformed_aruco_array = []

    for aruco in request.aruco_array: 
      if aruco.marker_ids not in self.done_aruco:
        transform = self.tf_buffer.lookup_transform(
          "base_link",
          "camera_color_optical_frame",
          Time(),
          timeout=rclpy.duration.Duration(seconds=1.0)
        )

        transformed = tf2_geometry_msgs.do_transform_pose(aruco.poses, transform)

        marker = ArucoMarkers()
        marker.marker_ids = aruco.marker_ids
        marker.poses = transformed

        response.transformed_aruco_array.append(marker)

    return response

  def handle_remove_request(self,request:RemoveAruco.Request, response:RemoveAruco.Response):

    id_2_remove = request.removed_aruco_id
    for i in self.transformed_aruco_array.aruco_array :
      if i.marker_ids == id_2_remove:
        self.transformed_aruco_array.aruco_array.remove(i)
        self.done_aruco.append(i.marker_ids)
        self.get_logger().info(f"The aruco_{id_2_remove} has been removed.")
        response.success = True
        break

      else:
        response.success = False

    return response

  def reset(self, request:Reset.Request, response: Reset.Response):

    if request.reset == True:
      self.transformed_aruco_array.aruco_array = []
      self.done_aruco = []
      self.aruco_array.aruco_array = []
      response.success = True
      self.get_logger().info("The aruco list has been reseted!")

    else:
       response = False
       self.get_logger().warn("Reset failed!")

    return response

def main(args=None):

  rclpy.init(args=args)
  aruco_node = ArucoNode()
  rclpy.spin(aruco_node)
  aruco_node.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
