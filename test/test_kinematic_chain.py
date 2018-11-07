#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState
from pd_motion_control.msg import *
from pd_motion_control.srv import *


class TestKinematicChain():

    def __int__(self):
        pass

    def __init__(self, topic_name=''):

        self.joint_states = JointState()
        rospy.Subscriber(topic_name, JointState, self.jointStateCallback)
        rospy.sleep(1.0)

    def fkService(self, service_name='', base_link='', joint_values=[]):
        """
            test computation of FK computation
            @:param service name: name used to call service
            @:param base_link: link name used for header, all transformation matrix generate accord this header frame
            @:param joint_values: requested joint value to compute fk
        """

        # create service client for fk computation
        rospy.wait_for_service(service_name)
        client = rospy.ServiceProxy(service_name, ComputeFK)

        # make service request
        # fill header for request
        request = ComputeFKRequest()
        request.header.frame_id = base_link
        request.header.stamp = rospy.Time().now()

        # fill joint state/values
        requested_joint_state = self.joint_states
        # no requested joint value than take current joint values
        if len(joint_values) is 0:
            pass

        else:
            # push requested joint values
            requested_joint_state.position = []
            for value in joint_values:
                requested_joint_state.position.append(value)
        request.robot_state = requested_joint_state

        # fill fk link names for request
        request.fk_link_names = []

        # send request for fk calculation
        response = client.call(request)

        # display results on the terminal
        if response.fk_success is True:
            rospy.loginfo(response.message)
        else:
            rospy.logerr(response.message)

        for fk_pose, link_name in map(None, response.fk_pose_stamped, response.fk_link_names):
            rospy.loginfo(link_name)
            rospy.loginfo(str(fk_pose))

        rospy.loginfo("eef value: \n %s", str(response.eef_pose_stamped))

    def jacobianService(self, service_name='', base_link='', joint_values=[]):

        """
            test computation of Jacobian matrix
            @:param service name: name used to call service
            @:param joint_values: requested joint value to compute jacobian
        """

        # create service client for fk computation
        rospy.wait_for_service(service_name)
        client = rospy.ServiceProxy(service_name, ComputeJacobian)

        # make service request
        # fill header for request
        request = ComputeJacobianRequest()

        # fill joint state/values
        requested_joint_state = self.joint_states
        # no requested joint value than take current joint values
        if len(joint_values) is 0:
            pass

        else:
            # push requested joint values
            requested_joint_state.position = []
            for value in joint_values:
                requested_joint_state.position.append(value)
        request.robot_state = requested_joint_state

        # send request for fk calculation
        response = client.call(request)

        # display results on the terminal
        if response.jac_success is True:
            rospy.loginfo(response.message)
        else:
            rospy.logerr(response.message)

        rospy.loginfo("jac_service: response: jac linear matrix")
        for jac_lin in response.jac_linear_vel:
            rospy.loginfo(jac_lin)

        rospy.loginfo("jac_service: response: jac angular matrix")
        for jac_ang in response.jac_angular_vel:
            rospy.loginfo(jac_ang)

    def jointStateCallback(self, msg):
        '''for i in range(0,len(msg.position)):
            self.joint_states[i] = msg.position[i]
        '''
        self.joint_states = msg


if __name__ == '__main__':
    rospy.init_node("test_kinematic_chain")
    rospy.loginfo("========================================")
    rospy.loginfo("Start testing kinematic chain node")
    rospy.loginfo("========================================")

    test = TestKinematicChain(topic_name='/arm/joint_state')

    # testing forward kinematics
    test.fkService(
        service_name='/arm/ComputeFK',
        base_link='base_link',
        joint_values=[0., 0., 0., 0., 0., 0.]
    )

    test.fkService(
        service_name='/arm/ComputeFK',
        base_link='base_link',
        joint_values=[-1.0202485703248891, -0.4086668184269392, -2.4022618507278723, -3.482983635056608, -1.8100613962049064, 0.0]
    )

    test.fkService(
        service_name='/arm/ComputeFK',
        base_link='base_link',
        joint_values=[-0.9002485703248891, -0.4086668184269392, -2.4022618507278723, -3.482983635056608, -1.7100613962049064, -0.0]
    )

    test.fkService(
        service_name='/arm/ComputeFK',
        base_link='base_link',
        joint_values=[1.8946927589099885, -1.2498882108994205, -1.4212811368752756, -3.621294495077807, -1.642768864585344, 0.0]
    )

    test.fkService(
        service_name='/arm/ComputeFK',
        base_link='base_link',
        joint_values=[2.2574481585254746, -1.8189895672673213, -1.8189896245843291, -3.9631237154760623, -1.6427599866715346, 0.0]
    )

    # testing jacobian matrix
    test.jacobianService(
        service_name='/arm/ComputeJacobian',
        base_link='base_link',
        joint_values=[0., 0., 0., 0., 0., 0.]
    )

    test.jacobianService(
        service_name='/arm/ComputeJacobian',
        base_link='base_link',
        joint_values=[-1.0202485703248891, -0.4086668184269392, -2.4022618507278723, -3.482983635056608, -1.8100613962049064, 0.0]
    )

    test.jacobianService(
        service_name='/arm/ComputeJacobian',
        base_link='base_link',
        joint_values=[-0.9002485703248891, -0.4086668184269392, -2.4022618507278723, -3.482983635056608, -1.7100613962049064, -0.0]
    )

    test.jacobianService(
        service_name='/arm/ComputeJacobian',
        base_link='base_link',
        joint_values=[1.8946927589099885, -1.2498882108994205, -1.4212811368752756, -3.621294495077807, -1.642768864585344, 0.0]
    )

    test.jacobianService(
        service_name='/arm/ComputeJacobian',
        base_link='base_link',
        joint_values=[2.2574481585254746, -1.8189895672673213, -1.8189896245843291, -3.9631237154760623, -1.6427599866715346, 0.0]
    )

    rospy.spin()