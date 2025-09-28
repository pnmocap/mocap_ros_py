import asyncio

from mocap_robotapi import *



class MCPMain():
    def __init__(self):   
        app = MCPApplication()
        settings = MCPSettings()
        settings.set_udp(7012)
        settings.set_bvh_rotation(0)
        app.set_settings(settings)
        app.open()
        
        while True:
            evts = app.poll_next_event()
            for evt in evts:
                if evt.event_type == MCPEventType.AvatarUpdated:
                    self.handleAvatar(evt.event_data.avatar_handle)
                 

    def handleAvatar(self, avatar_handle):
        # Handle avatar update event
        avatar = MCPAvatar(avatar_handle)  # Get avatar data
        joints = avatar.get_joints()  # Get all joint data
        str_data = '{'
        for joint in joints:
            link_name = joint.get_name()  # Get joint name
            position = joint.get_local_position()  # Get joint position
            rotation = joint.get_local_rotation()  # Get joint rotation
            str_data += f'{link_name} : {position}, {rotation}'
            str_data += '}'
        print(f"joints : {str_data}, ")




if __name__ == '__main__':
    MCPMain()