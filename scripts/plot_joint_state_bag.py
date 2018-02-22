from collections import namedtuple

import rosbag
import matplotlib.pyplot as plt

JointState = namedtuple('JointState', 'stamp, position, velocity')

bag = "/tmp/hypothesis_5.bag"
plot_joint_name = "l_shoulder_pan_joint"

joint_states = {}
for _, message, _ in rosbag.Bag(bag).read_messages("/joint_states"):
    for idx, joint_name in enumerate(message.name):
        if not joint_name in joint_states:
            joint_states[joint_name] = []
        joint_states[joint_name].append(JointState(stamp=message.header.stamp, position=message.position[idx], velocity=message.velocity[idx]))

print("Read %s joint states messages", len(joint_states["torso_lift_joint"]))

for joint_state in joint_states[plot_joint_name]:
    plt.scatter(joint_state[0].to_sec(), joint_state[1], s=5)

plt.title(plot_joint_name + " (" + bag + ")", fontsize=24)
plt.xlabel("time stamp", fontsize=14)
plt.ylabel("joint position", fontsize=14)

plt.show()