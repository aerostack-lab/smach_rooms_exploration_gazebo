#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import threading


import aerostack_msgs.srv as aersrv
import behavior_coordinator_msgs.srv as bcoordsrv
from behavior_execution_manager_msgs.msg import BehaviorActivationFinished 
from behavior_coordinator_msgs.msg import TaskCommand 

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

class EXECUTING_TAKE_OFF(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,outcomes=["take_off_completed"])
        self._trigger_event = threading.Event()
        


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
        call_service("/drone111/start_task",bcoordsrv.StartTask,bcoordsrv.StartTaskRequest(TaskCommand("TAKE_OFF", "", 1)))
        
        # once you have called the service, wait until take_off is completed
        # wait until internal variable has been set to true
        self._trigger_event.wait()
        
        self._sub.unregister()

        return "take_off_completed"


    def _cb(self,msg):

        if msg.name == "TAKE_OFF_WITH_PID":

            #sets the internal varible to true. If there were any thread waiting for this Event, they will get unblocked 
            self._trigger_event.set()


class EXECUTING_GENERATE_PATH(smach.State):


    def __init__(self,destination):

        smach.State.__init__(self,["path_generated","path_not_generated"])
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
        call_service("/drone111/start_task",bcoordsrv.StartTask,bcoordsrv.StartTaskRequest(TaskCommand("GENERATE_PATH", "destination: "+self.destination, 1)))
        
        # once you have called the service, wait until a new path has been generated
        # wait until internal variable has been set to true
        self._trigger_event.wait()
        
        self._sub.unregister()
        if self._path_generated :
            return "path_generated"
        else:
            return "path_not_generated"


    def _cb(self,msg):
        
        if msg.name == "GENERATE_PATH_WITH_OCCUPANCY_GRID":
            
            if msg.termination_cause == 1 :
                self._path_generated= True
            else :
                self._path_generated= False
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
        call_service("/drone111/start_task",bcoordsrv.StartTask,bcoordsrv.StartTaskRequest(TaskCommand("LAND", "", 1)))
        
        # once you have called the service, wait until land is completed
        # wait until internal variable has been set to true
        self._trigger_event.wait()
        
        self._sub.unregister()

        return "land_completed"

    def _cb(self,msg):

        if msg.name == "LAND_WITH_PID":

            #sets the internal varible to true. If there were any thread waiting for this Event, they will get unblocked
            self._trigger_event.set()

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

        self._follow_path_service_name="/drone111/start_task"
        self._follow_path_service_type=bcoordsrv.StartTask
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
        
            self._follow_path_service_request=bcoordsrv.StartTaskRequest(TaskCommand("FOLLOW_PATH", "path: "+self._path, 1))
            
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
            self._msg=msg

            #check if goal is achieved
            if msg.termination_cause != 1:
                self.goal=False
            else :
                self.goal=True 

            #sets the internal varible to true. If there were any thread waiting for this Event, they will get unblocked
            self._trigger_event.set()
    



def main():
    rospy.init_node('smach_example_state_machine')

    sm=smach.StateMachine(outcomes=['succeeded'])



    with sm:
        
        
        smach.StateMachine.add("EXECUTING_TAKE_OFF",EXECUTING_TAKE_OFF(),
                                    transitions={"take_off_completed":"EXECUTING_GENERATE_PATH_1"})
        
        smach.StateMachine.add("EXECUTING_GENERATE_PATH_1",EXECUTING_GENERATE_PATH("[-4.5,5,1]"),
                                    transitions={"path_generated":"EXECUTING_FOLLOW_PATH_1",
                                                    "path_not_generated":"EXECUTING_GENERATE_PATH_2"})
        
        smach.StateMachine.add("EXECUTING_FOLLOW_PATH_1",EXECUTING_FOLLOW_PATH(),
                                    transitions={"follow_path_completed":"EXECUTING_GENERATE_PATH_2",
                                                 "follow_path_failure":"EXECUTING_GENERATE_PATH_1"})
        
        smach.StateMachine.add("EXECUTING_GENERATE_PATH_2",EXECUTING_GENERATE_PATH("[0,0,1]"),
                                    transitions={"path_generated":"EXECUTING_FOLLOW_PATH_2",
                                                    "path_not_generated":"EXECUTING_LAND"})
        smach.StateMachine.add("EXECUTING_FOLLOW_PATH_2",EXECUTING_FOLLOW_PATH(),
                                    transitions={"follow_path_completed":"EXECUTING_LAND",
                                                 "follow_path_failure":"EXECUTING_GENERATE_PATH_2"})
        
        smach.StateMachine.add("EXECUTING_LAND",EXECUTING_LAND(),
                                    transitions={"land_completed":"succeeded"})
        
       
    
    sis = smach_ros.IntrospectionServer('smach_viewer', sm, '/SM_ROOT')
    
    sis.start()

    outcome = sm.execute() 

    rospy.spin()
    sis.stop()
    

if __name__ == '__main__':
    main()
