import json
import rospy
import tf

from sensor_msgs.msg import JointState, Joy


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


def asix_2_ros_translation(x, y, z):
    return (z, x, y)

def asix_2_ros_rotation(x, y, z, w):
    return (z, x, y, w)

def mocap_to_stickman_ros1():
    # init ros node
    rospy.init_node("real_time_transform_publisher", anonymous=True)
    rate = rospy.Rate(90)  # set fps to 90Hz

    json_file_path = './retarget.json'    
    robot = MCPRobot(open(json_file_path).read())
    app = MCPApplication()
    settings = MCPSettings()
    settings.set_udp(7012)
    settings.set_bvh_rotation(0)
    app.set_settings(settings)
    app.open()

    rospy.Publisher('/remoter/action_list', Joy, queue_size=1)
    br = tf.TransformBroadcaster()

    try:
        while not rospy.is_shutdown():
            evts = app.poll_next_event()
            for evt in evts:
                if evt.event_type == MCPEventType.AvatarUpdated:
                    avatar = MCPAvatar(evt.event_data.avatar_handle)
                    robot.update_robot(avatar)
                    robot.run_robot_step()

                    # get realtime data
                    real_time_data = json.loads(robot.get_robot_ros_frame_json()[0])
                    # print(json.dumps(real_time_data))

                    time_now = rospy.Time.now()

                    # avatar
                    # Traverse the link_poses and publish them to the TF (Transform) tree.
                    for link_name, pose_data in real_time_data["link_poses"].items():
                        translation = (pose_data[0], pose_data[1], pose_data[2])
                        rotation = (pose_data[3], pose_data[4], pose_data[5], -pose_data[6])
                        br.sendTransform(translation, rotation, time_now, link_name, links_parent[link_name])

                    # robot 
                    translation = asix_2_ros_translation(real_time_data["root_pos_x"], real_time_data["root_pos_y"], real_time_data["root_pos_z"]) 
                    rotation = asix_2_ros_rotation(real_time_data["root_rot_x"], real_time_data["root_rot_y"], real_time_data["root_rot_z"], real_time_data["root_rot_w"])
                    br.sendTransform(translation, rotation, time_now, "base_link", "world")

                
                elif evt.event_type == MCPEventType.RigidBodyUpdated:
                    print('rigid body updated')
                else:
                    print('unknow event')
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    app.close()
    
if __name__ == '__main__':    
    mocap_to_stickman_ros1()