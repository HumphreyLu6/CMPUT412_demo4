#!/usr/bin/env python
# BEGIN ALL
import rospy, numpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time
import smach
import smach_ros


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['navigating', 'wait', 'end'])
    
    def execute(self, userdata):
        global g_targets, g_start
        if rospy.is_shutdown:
            return 'end'

        if g_start == True:
            if len(g_targets) > 0:
                return 'navigating'
            else:
                print 'No targets, can not start.'
                return 'wait'
        else:
            return 'wait'

class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['navigating', 'arrived', 'end'])
    
    def execute(self, userdata):
        global g_start, g_targets
        if rospy.is_shutdown:
            return 'end'
        else:
            return 'end'

def joy_callback(msg):
    global g_targets, g_start
    if len(g_targets) <= 4:
        if msg.buttons[0] == 1:
            g_targets.append(1)
        if msg.buttons[1] == 1:
            g_targets.append(2)
        if msg.buttons[2] == 1:
            g_targets.append(3)
        if msg.buttons[3] == 1:
            g_targets.append(4)
    else:
        print "targets full !"      

    if msg.buttons[5] == 1:
        g_start = not g_start  
    
    print g_targets

def main():
    global g_targets, g_start
    g_targets = []
    g_start = False

    rospy.init_node("navi_bot")

    rospy.Subscriber("Joy", Joy, joy_callback)

    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        
        smach.StateMachine.add('Wait', Wait(),
                                transitions={'navigating':'Navigate',
                                             'end':'end',
                                             'wait':'Wait'})
        smach.StateMachine.add('Navigate', Navigate(),
                                transitions={'arrived':'Wait',
                                             'navigating':'Navigate'})
    outcome = sm.execute()

    rospy.spin()

if __name__ == "__main__":
    g_targets = None
    g_start = None
    main()

        