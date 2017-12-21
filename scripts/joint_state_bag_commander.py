#!/usr/bin/env python
import actionlib
import rosbag
import rospy
import control_msgs.msg
import trajectory_msgs.msg


class JointStateBagCommander(object):
    def __init__(self):
        self.filename = rospy.get_param('~filename')
        self.topic_name = "/joint_states" # TODO: from parameter server
        # TODO: from parameter server
        self.joint_names = ['torso_lift_joint', 'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint',
                            'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint',
                            'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint',
                            'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
        self.initial_move_duration = rospy.Duration(7.0) # TODO: from parameter server
        self.read_and_prepare_commands()
        self.client = actionlib.SimpleActionClient("/whole_body_controller/follow_joint_trajectory",
                                                   control_msgs.msg.FollowJointTrajectoryAction) # TODO: local namespace
        self.client.wait_for_server(rospy.Duration(1.0))

    def read_and_prepare_commands(self):
        # reading joint state messages into a list
        self.joint_states = []
        for topic, message, t in rosbag.Bag(self.filename).read_messages(self.topic_name):
            self.joint_states.append(message)
        rospy.loginfo("Read %s joint states messages", len(self.joint_states))

        # preparing joint trajectory goals
        self.goals = []
        if len(self.joint_states) > 1:
            self.goals.append(self.to_trajectory_msg(self.joint_states[0:1], self.initial_move_duration))
            #self.goals.append(self.to_trajectory_msg(self.joint_states[1:])) # TODO: uncomment me

    def to_trajectory_msg(self, joint_states, duration_offset=rospy.Duration(0.0)):
        result = control_msgs.msg.FollowJointTrajectoryGoal()
        result.trajectory.joint_names = self.joint_names

        start_time = self.joint_states[0].header.stamp

        # create trajectory point map, ordered by time from start
        point_map = {}
        for joint_state in joint_states:
            # calculate correct time from start for each trajectory point
            time_from_start = duration_offset + (joint_state.header.stamp - start_time)
            point_map[time_from_start] = {}
            for idx, joint_name in enumerate(joint_state.name):
                # only copy relevant joints
                if joint_name in self.joint_names:
                    point_map[time_from_start][joint_name] = {}
                    point_map[time_from_start][joint_name]["position"] = joint_state.position[idx]

        # fill result from map




        # TODO: complete me
        return result


    def run(self):
        for goal in self.goals:
            goal.trajectory.header.stamp = rospy.get_rostime()
            self.client.send_goal(goal)
            self.client.wait_for_result()
            if self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                raise Exception("Motion execution failed. Received result msg '%s'", self.client.get_result())

if __name__ == '__main__':
    rospy.init_node("joint_state_bag_comdr")
    my_comdr = JointStateBagCommander()
    my_comdr.run()
