#!/usr/bin/env python
import actionlib
import rosbag
import rospy
import control_msgs.msg
import std_srvs.srv
import trajectory_msgs.msg


class JointStateBagCommander(object):
    def __init__(self):
        self.filename = rospy.get_param('~filename')
        self.topic_name = rospy.get_param('~topic_name')
        self.joint_names = rospy.get_param('~joint_names')
        self.initial_move_duration = rospy.Duration(rospy.get_param('~init_move_duration'))
        self.read_and_prepare_commands()
        self.client = actionlib.SimpleActionClient("/whole_body_controller/follow_joint_trajectory",
                                                   control_msgs.msg.FollowJointTrajectoryAction) # TODO: local namespace
        self.client.wait_for_server(rospy.Duration(1.0))
        self.init_move_service = rospy.Service("~init_move", std_srvs.srv.Trigger, self.init_move)
        self.move_service = rospy.Service("~move", std_srvs.srv.Trigger, self.move)

    def read_and_prepare_commands(self):
        # reading joint state messages into a list
        self.joint_states = []
        for topic, message, t in rosbag.Bag(self.filename).read_messages(self.topic_name):
            self.joint_states.append(message)
        rospy.loginfo("Read %s joint states messages", len(self.joint_states))

        # preparing joint trajectory goals
        self.init_move_goal = None
        self.move_goal = None
        if len(self.joint_states) > 1:
            self.init_move_goal = self.to_trajectory_msg(self.joint_states[0:1], self.initial_move_duration)
            self.move_goal = self.to_trajectory_msg(self.joint_states)

    def to_trajectory_msg(self, joint_states, duration_offset=rospy.Duration(0.0)):
        result = control_msgs.msg.FollowJointTrajectoryGoal()
        result.trajectory.joint_names = self.joint_names

        start_time = joint_states[0].header.stamp

        # create joint state maps, ordered by time from start
        joint_state_maps = []
        for index, joint_state in enumerate(joint_states):
            # calculate correct time from start for each trajectory point
            time_from_start = (duration_offset + (joint_state.header.stamp - start_time))
            joint_state_map = {}
            joint_state_map["time_from_start"] = time_from_start
            for idx, joint_name in enumerate(joint_state.name):
                # only copy relevant joints
                if joint_name in self.joint_names:
                    joint_state_map[joint_name] = {}
                    joint_state_map[joint_name]["position"] = joint_state.position[idx]
            joint_state_maps.append(joint_state_map)

        for joint_state_map in joint_state_maps:
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.time_from_start = joint_state_map["time_from_start"]
            # preserve order of joint names given from user on parameter server
            for joint_name in result.trajectory.joint_names:
                point.positions.append(joint_state_map[joint_name]["position"])
            result.trajectory.points.append(point)

        return result

    def init_move(self, req):
        if self.init_move_goal == None:
            return std_srvs.srv.TriggerResponse(False, "Did not find a goal for init move in bag.")

        flag, message = self.send_goal(self.init_move_goal)
        return std_srvs.srv.TriggerResponse(flag, message)

    def move(self, req):
        if self.move_goal == None:
            return std_srvs.srv.TriggerResponse(False, "Did not find a goal for move in bag.")

        flag, message = self.send_goal(self.move_goal)
        return std_srvs.srv.TriggerResponse(flag, message)

    def send_goal(self, goal):
        goal.trajectory.header.stamp = rospy.get_rostime()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        flag = self.client.get_state() == actionlib.GoalStatus.SUCCEEDED
        message = ""
        if not flag:
            message = "Motion execution failed. Received result msg '%s'" % self.client.get_result()
        return flag, message

if __name__ == '__main__':
    rospy.init_node("joint_state_bag_comdr")
    my_comdr = JointStateBagCommander()
    rospy.spin()
