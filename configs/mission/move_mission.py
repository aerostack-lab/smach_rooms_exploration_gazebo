#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import threading


import aerostack_msgs.srv as aersrv
import behavior_execution_manager_msgs.srv as bsrv
from behavior_execution_manager_msgs.msg import BehaviorActivationFinished 

from SmachStates import *


def call_service(service_name,service_spec,service_request):
        proxy=None
        response=None
        while proxy is None:
            if rospy.is_shutdown():
                return None
            try:
                rospy.wait_for_service(service_name,1.0)
                proxy = rospy.ServiceProxy(service_name, service_spec)
            except rospy.ROSException as ex:
                    rospy.logwarn("Still waiting for service '%s'..." % service_name)

        #Call service
        try:
            rospy.logwarn("Calling service %s with request:\n%s" % (service_name, str(service_request)))
            response = proxy(service_request)
        except rospy.ServiceException as ex:
            rospy.logerr("Exception when calling service '%s': %s" % (self._service_name, str(ex)))
            return None

        return response
        
def convertPathtoList(pathToConvert):
    #auxiliar function for getPath method in FollowPath Class
    cont=0

    pathList="["
    path="["
    arrPathList=[]
            

    while cont < len(pathToConvert):
        if pathToConvert[cont] == "("  :
                    pass    
        elif pathToConvert[cont]== ")" and not path =="[":
            path=path+"]"
            arrPathList.append(path)
            pathList= pathList +path +","
            path="["
        
        elif pathToConvert[cont] == "," :
            if path != "[":
                path=path + ","
        else:
            path=path+pathToConvert[cont]

        cont=cont+1


    pathList=pathList[:-1]+ "]"

    return pathList





class EXECUTING_FOLLOW_PATH(smach.State):
    

    def __init__(self):

        smach.State.__init__(self,["follow_path_completed",
        "follow_path_failure"])

        self._follow_path_service_name="/drone111/quadrotor_motion_with_pid_control/behavior_follow_path/activate_behavior"
        self._follow_path_service_type=bsrv.ActivateBehavior
        self._follow_path_service_request=None
        self._activation_fin_topic_name="/drone111/behavior_activation_finished"
        self._activation_fin_topic_type=BehaviorActivationFinished
        self._path=[]
        self.goal=False
        self._query_service_name="/drone111/query_belief"
        self._query_service_type=aersrv.QueryBelief
        self._query_service_request=None
        self._query_path= "path(?x,?y)"
        self._remove_query_srv_name="/drone111/remove_belief"
        self._remove_query_srv_type=aersrv.RemoveBelief
        self._remove_query_srv_request=None
        self._counter=2
        self._remove_query_path="path("
        self._remove_query_object="object(2, path)"

        self._trigger_event = threading.Event()


    def execute(self,ud):
        
        self.goal= False

        self._path=self.getPath()
        
        #if there is a path
        if self._path:

            # declare subscriber
            self._sub=rospy.Subscriber(self._activation_fin_topic_name,self._activation_fin_topic_type,self._cb)

            """
            Event has an internal variable, if this variable is set to false when a call to wait occurs, the you will have to wait until 
            the internal variable is set to to true inside another thread.
            the method clear sets the internal variable to false 
            """
            self._trigger_event.clear()
        
            self._follow_path_service_request=bsrv.ActivateBehaviorRequest("path: "+self._path,1000)
            
            call_service(self._follow_path_service_name,self._follow_path_service_type,self._follow_path_service_request)
        
            #this state waits until follow_path ends.
            # wait until internal variable has been set to true
            self._trigger_event.wait()

            self._sub.unregister()

        #check if goal is achieved
        if(self.goal):
            return "follow_path_completed"
        else:
            return "follow_path_failure"

    
    def getPath(self):

        self._query_service_request=aersrv.QueryBeliefRequest(self._query_path)
        
        response=call_service(self._query_service_name,
                                            self._query_service_type,
                                            self._query_service_request)

        pathList=""


        if response.success:
            y= response.substitutions.split("y: ")[1]
            x=response.substitutions.split("y: ")[0].split(" ")[1]
            
            pathList= convertPathtoList(y)
            self._remove_query_path_f= self._remove_query_path +x+", "+ y +")"
            self._remove_query_object_f="object("+x+", path)"
            print(self._remove_query_object_f)
            print(self._remove_query_path_f)
            
            self._remove_query_srv_request= aersrv.RemoveBeliefRequest(self._remove_query_path_f)
            respuesta=call_service(self._remove_query_srv_name ,
                                            self._remove_query_srv_type,
                                            self._remove_query_srv_request)
            if respuesta.success :
                print("BORRADA")
            self._remove_query_srv_request=aersrv.RemoveBeliefRequest(self._remove_query_object_f)
            call_service(self._remove_query_srv_name ,
                                            self._remove_query_srv_type,
                                            self._remove_query_srv_request)
        
        return pathList



    def _cb(self,msg):
        
        if msg.name == "FOLLOW_PATH":
            
            #check if goal is achieved
            if msg.termination_cause != 1:
                self.goal=False
            else :
                self.goal=True 

            #sets the internal varible to true. If there were any thread waiting for this Event, they will get unblocked 
            self._trigger_event.set()




class EXECUTING_GENERATE_PATH(smach.State):
    #This state is going to execute simultaneously to Hover State.
    #It will call the generate path behaviour, and will block until
    #a new path has been published .
    def __init__(self,destination):

        smach.State.__init__(self,["path_generated"])
        self._trigger_event = threading.Event()
        self.destination=destination

    def execute(self,ud):

        # declare subscriber
        self._sub = rospy.Subscriber("/drone111/behavior_activation_finished", BehaviorActivationFinished, self._cb)

        """
        Event has an internal variable, if this variable is set to false when a call to wait occurs, then you will have to wait until 
        the internal variable is set to to true inside another thread.
        the method clear sets the internal variable to false 
        """
        self._trigger_event.clear()
        
        #call_service calls the service specify in the first argumentent with the typpe of msg specify in the second argumentent
        #and with the request specify in the third argument
        call_service("/drone111/navigation_with_lidar/behavior_generate_path_with_occupancy_grid/activate_behavior",bsrv.ActivateBehavior,bsrv.ActivateBehaviorRequest("destination: "+self.destination,1000))
        
        # once you have called the service, wait until a new path has been generated
        # wait until internal variable has been set to true
        self._trigger_event.wait()
        
        self._sub.unregister()

        return "path_generated"

        
        

    def _cb(self,msg):
    
        if msg.name == "GENERATE_PATH_WITH_OCCUPANCY_GRID":

            if msg.termination_cause == 1 :
                self._path_generated= True
            else :
                self._path_generated= False

            #sets the internal varible to true. If there were any thread waiting for this Event, they will get unblocked 
            self._trigger_event.set()






class EXECUTING_TAKE_OFF(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=["take_off_completed"])
        self._trigger_event = threading.Event()
        


    def execute(self,ud):
        # declare subscriber
        self._sub = rospy.Subscriber("/drone111/behavior_activation_finished",
         BehaviorActivationFinished, self._cb)

        """
        Event has an internal variable, if this variable is set to false when a call to wait occurs, then you will have to wait until 
        the internal variable is set to to true inside another thread.
        the method clear sets the internal variable to false 
        """
        self._trigger_event.clear()
        
        #call_service calls the service specify in the first argumentent with the typpe of msg specify in the second argumentent
        #and with the request specify in the third argument
        call_service("/drone111/quadrotor_motion_with_pid_control/behavior_take_off_with_pid/activate_behavior",
        bsrv.ActivateBehavior,bsrv.ActivateBehaviorRequest("",1000))
        
        # once you have called the service, wait until take_off is completed
        # wait until internal variable has been set to true
        self._trigger_event.wait()
        
        self._sub.unregister()

        return "take_off_completed"

    def _cb(self,msg):
        
        #sets the internal varible to true. If there were any thread waiting for this Event, they will get unblocked 
        self._trigger_event.set()

class EXECUTING_LAND(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=["land_completed"])
        self._trigger_event = threading.Event()


    def execute(self,ud):

        # declare subscriber
        self._sub = rospy.Subscriber("/drone111/behavior_activation_finished", BehaviorActivationFinished, self._cb)

        """
        Event has an internal variable, if this variable is set to false when a call to wait occurs, the you will have to wait until 
        the internal variable is set to to true inside another thread.
        the method clear sets the internal variable to false 
        """
        self._trigger_event.clear()
        
        #call_service calls the service specify in the first argumentent with the typpe of msg specify in the second argumentent
        #and with the request specify in the third argument
        call_service("/drone111/quadrotor_motion_with_pid_control/behavior_land_with_pid/activate_behavior"
        ,bsrv.ActivateBehavior,bsrv.ActivateBehaviorRequest("",1000))
        
        # once you have called the service, wait until land is completed
        # wait until internal variable has been set to true
        self._trigger_event.wait()
        
        self._sub.unregister()

        return "land_completed"

    def _cb(self,msg):
        
         #sets the internal varible to true. If there were any thread waiting for this Event, they will get unblocked
        self._trigger_event.set()



class ACTIVATING_HOVER(ServiceState):

    def __init__(self):

        ServiceState.__init__(self,"/drone111/quadrotor_motion_with_pid_control/behavior_keep_hovering_with_pid_control/activate_behavior",
                                    bsrv.ActivateBehavior,
                                    bsrv.ActivateBehaviorRequest("",1000),
                                    outcomes=['hover_activated'])
    
    def execute(self,ud):
        
        """
        ServiceState is a child from smach.State, this state is thought as an state for just calling a service
        ServiceState.execute will call the service that we passed on the init
        """
        res=ServiceState.execute(self,ud)
        if res == "succeeded" :
            return "hover_activated"
        else :
            return res 

class DEACTIVATING_HOVER(ServiceState):

    def __init__(self):

        """
        ServiceState is a child from smach.State, this state is thought as an state for just calling a service
        ServiceState.execute will call the service that we passed on the init
        """
        ServiceState.__init__(self,"/drone111/quadrotor_motion_with_pid_control/behavior_keep_hovering_with_pid_control/deactivate_behavior",
                                    bsrv.DeactivateBehavior,
                                    bsrv.DeactivateBehaviorRequest(),
                                    outcomes=['hover_deactivated'])
    
    def execute(self,ud):
        res=ServiceState.execute(self,ud)
        if res == "succeeded" :
            return "hover_deactivated"
        else :
            return res 


def main():
    rospy.init_node('smach_example_state_machine')

    sm=smach.StateMachine(outcomes=['succeeded','aborted','preempted'])



    with sm:
        
        
        smach.StateMachine.add("ACTIVATING_SELF_LOCALIZE",ACTIVATING_SELF_LOCALIZE(),
                                    transitions={"self_localize_activated":"ACTIVATING_THRUST_CONTROL"})
        
        smach.StateMachine.add("ACTIVATING_THRUST_CONTROL",ACTIVATING_THRUST_CONTROL(),
                                    transitions={"thrust_control_activated":"ACTIVATING_PID_MOTION_CONTROL"})
        smach.StateMachine.add("ACTIVATING_PID_MOTION_CONTROL",ACTIVATING_PID_MOTION_CONTROL(),
                                    transitions={"pid_motion_control_activated":"EXECUTING_TAKE_OFF"})
        smach.StateMachine.add("EXECUTING_TAKE_OFF",EXECUTING_TAKE_OFF(),
                                    transitions={"take_off_completed":"ACTIVATING_HOVER_1"})
        
        smach.StateMachine.add("ACTIVATING_HOVER_1",ACTIVATING_HOVER(),
                                    transitions={"hover_activated":"EXECUTING_GENERATE_PATH_1"})
        
        smach.StateMachine.add("EXECUTING_GENERATE_PATH_1",EXECUTING_GENERATE_PATH("[-4.5,5,1]"),
                                    transitions={"path_generated":"DEACTIVATING_HOVER_1"})
        
        smach.StateMachine.add("DEACTIVATING_HOVER_1",DEACTIVATING_HOVER(),
                                    transitions={"hover_deactivated":"EXECUTING_FOLLOW_PATH_1"})
        smach.StateMachine.add("EXECUTING_FOLLOW_PATH_1",EXECUTING_FOLLOW_PATH(),
                                    transitions={"follow_path_completed":"ACTIVATING_HOVER_2",
                                                 "follow_path_failure":"ACTIVATING_HOVER_1"})
        
                      
        smach.StateMachine.add("ACTIVATING_HOVER_2",ACTIVATING_HOVER(),
                                    transitions={"hover_activated":"EXECUTING_GENERATE_PATH_2"})
        smach.StateMachine.add("EXECUTING_GENERATE_PATH_2",EXECUTING_GENERATE_PATH("[0,0,1]"),
                                    transitions={"path_generated":"DEACTIVATING_HOVER_2"})
        smach.StateMachine.add("DEACTIVATING_HOVER_2",DEACTIVATING_HOVER(),
                                    transitions={"hover_deactivated":"EXECUTING_FOLLOW_PATH_2"})
        smach.StateMachine.add("EXECUTING_FOLLOW_PATH_2",EXECUTING_FOLLOW_PATH(),
                                    transitions={"follow_path_completed":"EXECUTING_LAND",
                                                 "follow_path_failure":"ACTIVATING_HOVER_2"})
        smach.StateMachine.add("EXECUTING_LAND",EXECUTING_LAND(),
                                    transitions={"land_completed":"DEACTIVATING_PID_MOTION_CONTROL"})
        smach.StateMachine.add("DEACTIVATING_PID_MOTION_CONTROL",DEACTIVATING_PID_CONTROL(),
                                    transitions={"pid_motion_control_deactivated":"DEACTIVATING_THRUST_CONTROL"})
        smach.StateMachine.add("DEACTIVATING_THRUST_CONTROL",DEACTIVATING_THRUST_CONTROL(),
                                    transitions={"thrust_control_deactivated":"DEACTIVATING_SELF_LOCALIZE"})
        smach.StateMachine.add("DEACTIVATING_SELF_LOCALIZE",DEACTIVATING_SELF_LOCALIZE(),
                                    transitions={"self_localize_deactivated":"succeeded"})
        

          
    
    sis = smach_ros.IntrospectionServer('smach_viewer', sm, '/SM_ROOT')
    
    sis.start()

    outcome = sm.execute()  

    rospy.spin()
    sis.stop()
    

if __name__ == '__main__':
    main()