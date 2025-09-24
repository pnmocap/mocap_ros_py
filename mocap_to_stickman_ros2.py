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
    "RightTiptoe": "RightFoot",
    "LeftUpLeg": "Hips",
    "LeftLeg": "LeftUpLeg",
    "LeftFoot": "LeftLeg",
    "LeftTiptoe": "LeftFoot",
    "Spine": "Hips",
    "Spine1": "Spine",
    "Spine2": "Spine1",
    "Neck": "Spine2",
    "Neck1": "Neck",
    "Head": "Neck1",
    "Head1": "Head",
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
    "RightHandPinky1": "RightHandPinky",
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
    "LeftHandPinky1": "LeftHandPinky",
    "LeftHandPinky2": "LeftHandPinky1",
    "LeftHandPinky3": "LeftHandPinky2"
}
def mocap_to_stickman_ros2():       
  try:
    rclpy.init()
    node = Node("real_time_transform_publisher")
    node.create_timer(0.1, lambda: send_link_poses_tf(node))
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
                #   avatar = MCPAvatar(evt.event_data.avatar_handle)
                  handleAvatar(br, node, evt.event_data.avatar_handle)
              
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

def handleAvatar(br, node, avatar_handle):
   
        # Handle avatar update event
        avatar = MCPAvatar(avatar_handle)  # Get avatar data

        # Get all joint data
        joints = avatar.get_joints()
        for joint in joints:
            link_name = joint.get_name()
            position = joint.get_local_position()
            rotation = joint.get_local_rotation()

            # Create and send transform for each joint
            send_transform(br, node, link_name, position, rotation)

        # Head1 
        rotation = (0.0, 0.0, 0.0, 1.0) 
        position = (0.000000, 16.450001, 0.000000)
        send_transform(br, node, 'Head1', position, rotation)

        # LeftTiptoe RightTiptoe
        position = (0.000000, -7.850000, 14.280000)
        send_transform(br, node, 'LeftTiptoe', position, rotation)
        send_transform(br, node, 'RightTiptoe', position, rotation)

        # Handle the root joint of the robot
        root_joint = avatar.get_root_joint()
        position = root_joint.get_local_position()
        rotation = root_joint.get_local_rotation()

        # Publish the root joint transform
        send_transform(br, node, 'base_link', position, rotation, frame_id='world')



def send_transform(br, node, child_frame_id, position, rotation, frame_id=None):
    
        if frame_id is None:
            frame_id = links_parent[child_frame_id]

        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        # Set translation
        t.transform.translation.x = position[2] / 100
        t.transform.translation.y = position[0] / 100
        t.transform.translation.z = position[1] / 100

        # Set rotation
        t.transform.rotation.x = rotation[3]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[0]
        # print(t.child_frame_id, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        br.sendTransform(t)

if __name__ == '__main__':
  mocap_to_stickman_ros2()