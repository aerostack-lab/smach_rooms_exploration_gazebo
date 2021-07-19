#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import threading

from smach_ros import ServiceState
from smach_ros import MonitorState

import behavior_execution_manager_msgs.srv as bsrv
from behavior_execution_manager_msgs.msg import BehaviorActivationFinished 




class ACTIVATING_SELF_LOCALIZE(ServiceState):
    def __init__(self):

        ServiceState.__init__(self,"drone111/basic_quadrotor_behaviors/behavior_self_localize_with_ground_truth/activate_behavior",
                            bsrv.ActivateBehavior,
                            request=bsrv.ActivateBehaviorRequest("",1000),
                            outcomes=['self_localize_activated'])
    
    def execute(self,ud):

        """
        ServiceState is a child from smach.State, this state is thought as an state for just calling a service
        ServiceState.execute will call the service that we passed on the init
        """
        res=ServiceState.execute(self,ud)
        if res == "succeeded" :
            return "self_localize_activated"
        else :
            return res 

class ACTIVATING_THRUST_CONTROL(ServiceState):

    def __init__(self):

        ServiceState.__init__(self,"/drone111/quadrotor_motion_with_pid_control/behavior_quadrotor_pid_thrust_control/activate_behavior",
                            bsrv.ActivateBehavior,
                            request=bsrv.ActivateBehaviorRequest("",1000),
                            outcomes=['thrust_control_activated'])
    
    def execute(self,ud):

        """
        ServiceState is a child from smach.State, this state is thought as an state for just calling a service
        ServiceState.execute will call the service that we passed on the init
        """
        res=ServiceState.execute(self,ud)
        if res == "succeeded" :
            return "thrust_control_activated"
        else :
            return res 

class ACTIVATING_PID_MOTION_CONTROL(ServiceState):

    def __init__(self):

        ServiceState.__init__(self,"/drone111/quadrotor_motion_with_pid_control/behavior_quadrotor_pid_motion_control/activate_behavior",
                            bsrv.ActivateBehavior,
                            request=bsrv.ActivateBehaviorRequest("",1000),
                            outcomes=["pid_motion_control_activated"])
    def execute(self,ud):

        """
        ServiceState is a child from smach.State, this state is thought as an state for just calling a service
        ServiceState.execute will call the service that we passed on the init
        """
        res=ServiceState.execute(self,ud)
        if res == "succeeded" :
            return "pid_motion_control_activated"
        else :
            return res 

class DEACTIVATING_PID_CONTROL(ServiceState):

    def __init__(self):

        ServiceState.__init__(self,"/drone111/quadrotor_motion_with_pid_control/behavior_quadrotor_pid_motion_control/deactivate_behavior",
                            bsrv.DeactivateBehavior,
                            request=bsrv.DeactivateBehaviorRequest(),
                            outcomes=["pid_motion_control_deactivated"])
        
    
    def execute(self,ud):

        """
        ServiceState is a child from smach.State, this state is thought as an state for just calling a service
        ServiceState.execute will call the service that we passed on the init
        """
        res=ServiceState.execute(self,ud)
        if res == "succeeded" :
            return "pid_motion_control_deactivated"
        else :
            return res
class DEACTIVATING_THRUST_CONTROL(ServiceState):

    def __init__(self):

        ServiceState.__init__(self,"/drone111/quadrotor_motion_with_pid_control/behavior_quadrotor_pid_thrust_control/deactivate_behavior",
                            bsrv.DeactivateBehavior,
                            request=bsrv.DeactivateBehaviorRequest(),
                            outcomes=["thrust_control_deactivated"])
        
    
    def execute(self,ud):

        """
        ServiceState is a child from smach.State, this state is thought as an state for just calling a service
        ServiceState.execute will call the service that we passed on the init
        """
        res=ServiceState.execute(self,ud)
        if res == "succeeded" :
            return "thrust_control_deactivated"
        else :
            return res

class DEACTIVATING_SELF_LOCALIZE(ServiceState):

    def __init__(self):

        ServiceState.__init__(self,"drone111/basic_quadrotor_behaviors/behavior_self_localize_with_ground_truth/deactivate_behavior",
                            bsrv.DeactivateBehavior,
                            request=bsrv.DeactivateBehaviorRequest(),
                            outcomes=['self_localize_deactivated'])
    
    def execute(self,ud):

        """
        ServiceState is a child from smach.State, this state is thought as an state for just calling a service
        ServiceState.execute will call the service that we passed on the init
        """
        res=ServiceState.execute(self,ud)
        if res == "succeeded" :
            return "self_localize_deactivated"
        else :
            return res 