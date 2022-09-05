#!/usr/bin/env python
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('target_pub')
        # List of goal poses:
        self.pose_seq = list()

        self.points_sub = rospy.Subscriber(
            'target_poses', Float32MultiArray, self.callback, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("target_pub node initialized!")

        rospy.spin()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1) +
                      " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        pass
        # To print current pose at each feedback:
        # rospy.loginfo("Feedback for goal pose " +
        #               str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt) +
                          " received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose " +
                              str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(
                    next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt) +
                          " was aborted by the Action Server")
            rospy.signal_shutdown(
                "Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt) +
                          " has been rejected by the Action Server")
            rospy.signal_shutdown(
                "Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt) +
                          " received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " +
                      str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb,
                              self.active_cb, self.feedback_cb)

    def callback(self, msg):
        self.goal_cnt = 0

        points_seq = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
        for point in points_seq:
            self.pose_seq.append(
                Pose(point, quaternion_from_euler(0, 0, 0, axes='sxyz')))  # 默认姿态

        self.movebase_client()


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
