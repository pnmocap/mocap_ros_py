import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from mocap_robotapi import *

links_parent = {
    "Hips": "world",
    "RightUpLeg": "Hips",
    "RightLeg": "RightUpLeg",
    "RightFoot": "RightLeg",
    "LeftUpLeg": "Hips",
    "LeftLeg": "LeftUpLeg",
    "LeftFoot": "LeftLeg",
    "Spine": "Hips",
    "Spine1": "Spine",
    "Spine2": "Spine1",
    "Neck": "Spine2",
    "Neck1": "Neck",
    "Head": "Neck1",
    "RightShoulder": "Spine2",
    "RightArm": "RightShoulder",
    "RightForeArm": "RightArm",
    "RightHand": "RightForeArm",
    "RightHandThumb1": "RightHand",
    "RightHandThumb2": "RightHandThumb1",
    "RightHandThumb3": "RightHandThumb2",
    "RightInHandIndex": "RightHand",
    "RightHandIndex1": "RightInHandIndex",
    "RightHandIndex2": "RightHandIndex1",
    "RightHandIndex3": "RightHandIndex2",
    "RightInHandMiddle": "RightHand",
    "RightHandMiddle1": "RightInHandMiddle",
    "RightHandMiddle2": "RightHandMiddle1",
    "RightHandMiddle3": "RightHandMiddle2",
    "RightInHandRing": "RightHand",
    "RightHandRing1": "RightInHandRing",
    "RightHandRing2": "RightHandRing1",
    "RightHandRing3": "RightHandRing2",
    "RightInHandPinky": "RightHand",
    "RightHandPinky1": "RightInHandPinky",
    "RightHandPinky2": "RightHandPinky1",
    "RightHandPinky3": "RightHandPinky2",
    "LeftShoulder": "Spine2",
    "LeftArm": "LeftShoulder",
    "LeftForeArm": "LeftArm",
    "LeftHand": "LeftForeArm",
    "LeftHandThumb1": "LeftHand",
    "LeftHandThumb2": "LeftHandThumb1",
    "LeftHandThumb3": "LeftHandThumb2",
    "LeftInHandIndex": "LeftHand",
    "LeftHandIndex1": "LeftInHandIndex",
    "LeftHandIndex2": "LeftHandIndex1",
    "LeftHandIndex3": "LeftHandIndex2",
    "LeftInHandMiddle": "LeftHand",
    "LeftHandMiddle1": "LeftInHandMiddle",
    "LeftHandMiddle2": "LeftHandMiddle1",
    "LeftHandMiddle3": "LeftHandMiddle2",
    "LeftInHandRing": "LeftHand",
    "LeftHandRing1": "LeftInHandRing",
    "LeftHandRing2": "LeftHandRing1",
    "LeftHandRing3": "LeftHandRing2",
    "LeftInHandPinky": "LeftHand",
    "LeftHandPinky1": "LeftInHandPinky",
    "LeftHandPinky2": "LeftHandPinky1",
    "LeftHandPinky3": "LeftHandPinky2"
}
def mocap_to_stickman_ros2():       
  try:
    rclpy.init()
    node = Node("real_time_transform_publisher")
    node.create_timer(0.1, lambda: send_link_poses_tf(node))
    json_file_path = './retarget.json'
    robot = MCPRobot(open(json_file_path).read())
    app = MCPApplication()
    settings = MCPSettings()
    settings.set_udp(7012)
    settings.set_bvh_rotation(0)
    app.set_settings(settings)
    app.open()
    
    br = StaticTransformBroadcaster(node)
    def send_link_poses_tf(node):
          evts = app.poll_next_event()
          for evt in evts:
              if evt.event_type == MCPEventType.AvatarUpdated:
                  avatar = MCPAvatar(evt.event_data.avatar_handle)
                  robot.update_robot(avatar)
                  robot.run_robot_step()
                  #print (robot.get_robot_ros_frame_json())
                  
                  # get realtime data
                  real_time_data = json.loads(robot.get_robot_ros_frame_json()[0])

                  # publish TF
                  for link_name, pose_data in real_time_data["link_poses"].items():
                        t = TransformStamped()
                        t.header.stamp = node.get_clock().now().to_msg()
                        t.header.frame_id =  links_parent[link_name]
                        t.child_frame_id = link_name
                        # translation
                        t.transform.translation.x = pose_data[0]
                        t.transform.translation.y = pose_data[1]
                        t.transform.translation.z = pose_data[2]

                        # rotation
                        #quat = tf_transformations.quaternion_from_euler(0, 0, 0)
                        t.transform.rotation.x = pose_data[3]
                        t.transform.rotation.y = pose_data[4]
                        t.transform.rotation.z = pose_data[5]
                        t.transform.rotation.w = pose_data[6]
                        br.sendTransform(t)
                    
                  # publish
                  t = TransformStamped()
                  t.header.stamp = node.get_clock().now().to_msg()
                  t.header.frame_id = 'world'
                  t.child_frame_id = 'base_link'

                  # translation
                  t.transform.translation.x = real_time_data["root_pos_x"]
                  t.transform.translation.y = real_time_data["root_pos_y"]
                  t.transform.translation.z = real_time_data["root_pos_z"]

                  # rotation
                  #quat = tf_transformations.quaternion_from_euler(0, 0, 0)
                  t.transform.rotation.x = real_time_data["root_rot_x"]
                  t.transform.rotation.y = real_time_data["root_rot_y"]
                  t.transform.rotation.z = real_time_data["root_rot_z"]
                  t.transform.rotation.w = real_time_data["root_rot_w"]
                  br.sendTransform(t)
              
              elif evt.event_type == MCPEventType.RigidBodyUpdated:
                  print('rigid body updated')
              else:
                  print('unknow event')    
    rclpy.spin(node)
  except Exception as e:
          node.get_logger().error(f"Error publishing joint state: {e}")      
  finally:
      app.close()
      node.destroy_node()
      rclpy.shutdown()  

if __name__ == '__main__':
  mocap_to_stickman_ros2()