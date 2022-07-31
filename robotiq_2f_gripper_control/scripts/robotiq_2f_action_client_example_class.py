#! /usr/bin/env python3
"""--------------------------------------------------------------------
This Node/Script shows an example on how to use a `SimpleActionClient` instance 
to control a Robotiq 2 Finger gripper.

Parameters:
    action_name: Name of the action advertised by the `ActionServer` controlling the gripper.

@author: Daniel Felipe Ordonez Apraez
@email: daniels.ordonez@gmail.com
--------------------------------------------------------------------"""

import rospy

# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_msgs.srv import Robotiq2FMove

from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq


class robotiq_operate():
    def __init__(self):
        self.action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.robotiq_client = actionlib.SimpleActionClient(self.action_name, CommandRobotiqGripperAction)

        # Wait until grippers are ready to take command
        self.robotiq_client.wait_for_server()
        ## Manually set all the parameters of the gripper goal state.
         ######################################################################################
        self.goal = CommandRobotiqGripperGoal()
        self.goal.emergency_release = False
        self.goal.stop = False
        self.goal.position = 0.70
        self.goal.speed = 0.1
        self.goal.force = 5.0
         
        # Sends the goal to the gripper.
        self.robotiq_client.send_goal(self.goal)
        # Block processing thread until gripper movement is finished, comment if waiting is not necesary.
        self.robotiq_client.wait_for_result()
    
        self.srv = rospy.Service("/robotiq_control_move",Robotiq2FMove,self.gripper_control_cb)
    
    def gripper_control_cb(self,req):
        Robotiq.goto(self.robotiq_client, pos=req.width, speed=0.1, force=100 , block=True)
        return True

def main():
    rospy.init_node("robotiq_gripper_control_node",anonymous=False)
    rate = rospy.Rate(10)
    
    operate = robotiq_operate()
    print("Ready to return")
    rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
