#! /usr/bin/env python

# Author: Tejaswi Digumarti (tejaswi.digumarti@sydney.edu.au)
# Description: This implements a hack to attach or detach two objects in gazebo. This code relies on the
#              gazebo_ros_link attacher package.

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachResponse, AttachRequest

class GazeboLinkAttacher(object):

    def __init__(self):
        super(GazeboLinkAttacher, self).__init__()
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.attach_srv.wait_for_service()

        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.detach_srv.wait_for_service()

    def attach_objects(self, model_name_1, model_name_2, link_name_1="link", link_name_2="link"):
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        self.attach_srv.call(req)

    def detach_objects(self, model_name_1, model_name_2, link_name_1="link", link_name_2="link"):
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        self.detach_srv.call(req)