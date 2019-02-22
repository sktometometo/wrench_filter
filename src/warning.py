#! /usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
import time
import sys
import os
import subprocess

from geometry_msgs.msg import WrenchStamped

def beep():
    """
    """
    filename = os.path.dirname( sys.argv[0] ) + "/beep.mp3"
    #os.system("mplayer " + filename)
    #subprocess.call(["mplayer" , filename, ">/dev/null", "2>&1"])
    subprocess.Popen("mplayer " + filename + " >/dev/null 2>&1", shell=True)

class WrenchWarning(object):
    
    def __init__( self, input_topic_name, node_name, wrench_max, beeprangeminrate=0.7 ):
        """
        """
        rospy.init_node( node_name ) 
        self.sub = rospy.Subscriber( input_topic_name, WrenchStamped, self.callback )
        self.wrench_max = wrench_max
        self.beeprangeminrate = beeprangeminrate

    def callback( self, msg ):
        """
        """
        print( "Fx rate : {:.1%}".format( abs(msg.wrench.force.x) / ( self.wrench_max[0]) ) \
              + " ," + "Fy rate : {:.1%}".format( abs(msg.wrench.force.y) / ( self.wrench_max[1]) ) \
              + " ," + "Fz rate : {:.1%}".format( abs(msg.wrench.force.z) / ( self.wrench_max[2]) ) \
              + " ," + "Tx rate : {:.1%}".format( abs(msg.wrench.torque.x) / ( self.wrench_max[3]) ) \
              + " ," + "Ty rate : {:.1%}".format( abs(msg.wrench.torque.y) / ( self.wrench_max[4]) ) \
              + " ," + "Tz rate : {:.1%}".format( abs(msg.wrench.torque.z) / ( self.wrench_max[5]) ) )

        if abs( msg.wrench.force.x ) > self.beeprangeminrate * self.wrench_max[0]:
            print("WARN: force x")
        if abs( msg.wrench.force.y ) > self.beeprangeminrate * self.wrench_max[1]:
            beep()
            print("WARN: force y")
        if abs( msg.wrench.force.z ) > self.beeprangeminrate * self.wrench_max[2]:
            beep()
            print("WARN: force z")
        if abs( msg.wrench.torque.x ) > self.beeprangeminrate * self.wrench_max[3]:
            beep()
            print("WARN: torque x")
        if abs( msg.wrench.torque.y ) > self.beeprangeminrate * self.wrench_max[4]:
            beep()
            print("WARN: torque y")
        if abs( msg.wrench.torque.z ) > self.beeprangeminrate * self.wrench_max[5]:
            beep()
            print("WARN: torque z")

if __name__ == "__main__":

    a = WrenchWarning( "/cfs/wrench",
                       "cfs_warning",
                       [ 250, 250, 250, 6, 6, 6 ],
                       1.0
                        )
    rospy.spin()
