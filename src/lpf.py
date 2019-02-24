#! /usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
import sys

from geometry_msgs.msg import WrenchStamped

class LPF(object):

    def __init__(self, input_topic, output_topic, lpf_factor, node_name="LPF"):
        """
        """

        rospy.init_node( node_name )
        
        self.sub = rospy.Subscriber( input_topic, WrenchStamped, self.callback )
        self.pub = rospy.Publisher( output_topic, WrenchStamped, queue_size=10 )
        self.lpf_factor = lpf_factor

        self.pre_F = np.zeros(3)
        self.pre_T =np.zeros(3)

        
    def callback( self, msg ):
        """
        """
        raw_F = np.array( [ msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z ] )   
        raw_T = np.array( [ msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z ] )

        pub_F = self.lpf_factor * raw_F + ( 1 - self.lpf_factor ) * self.pre_F
        pub_T = self.lpf_factor * raw_T + ( 1 - self.lpf_factor ) * self.pre_T

        self.pre_F = raw_F
        self.pre_T = raw_T

        publishdata = WrenchStamped()
        publishdata.header = msg.header
        publishdata.wrench.force.x = pub_F[0]
        publishdata.wrench.force.y = pub_F[1]
        publishdata.wrench.force.z = pub_F[2]
        publishdata.wrench.torque.x = pub_T[0]
        publishdata.wrench.torque.y = pub_T[1]
        publishdata.wrench.torque.z = pub_T[2]
        self.pub.publish( publishdata )

        print( "published. " + "F: " + str( pub_F ) + ", T: " + str( pub_T ) )

if __name__=="__main__":

    if len( sys.argv ) < 4:
        print( "usage: program <input topic name> <output topic name> <lpf_factor> <node name>" )
        sys.exit(1)

    a = LPF( sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4] )
    rospy.spin()
