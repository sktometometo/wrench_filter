#! /usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
import time
import sys

import wrench_filter
import wrench_filter.srv
from geometry_msgs.msg import WrenchStamped

gravitycoef = 9.80665

class WrenchFilter(object):

    def __init__( self, input_topic_name,\
                        output_topic_name,\
                        calib_service_name,\
                        node_name="WrenchFilter" ):
        """
        """

        rospy.init_node( node_name )
        self.sub = rospy.Subscriber( input_topic_name, WrenchStamped, self.callback )
        self.pub = rospy.Publisher( output_topic_name, WrenchStamped, queue_size=10 )
        self.service = rospy.Service( calib_service_name, wrench_filter.srv.Calibration, self.docalib )

        self.position = np.array( [ 0, 0, -51.1 ] ) # 6軸計測モジュールから見た, 6軸センサの姿勢
        self.R = np.array([[ 0.0,-1.0, 0.0 ],
                           [ 1.0, 0.0, 0.0 ],
                           [ 0.0, 0.0, 1.0 ]])    # 6軸計測モジュールから見た, 6軸センサの位置 

# FootX Configuration
#        self.position = np.array( [ -56, 0, 0 ] ) # 6軸計測モジュールから見た, 6軸センサの姿勢
#        self.R = np.array([[ 0.0, 0.0, 1.0 ],
#                           [ 0.0, 1.0, 0.0 ],
#                           [-1.0, 0.0, 0.0 ]])    # 6軸計測モジュールから見た, 6軸センサの位置 

# FootY Configuration
#        self.position = np.array( [ 0, 0, -51.1 ] ) # 6軸計測モジュールから見た, 6軸センサの姿勢
#        self.R = np.array([[ 0.0,-1.0, 0.0 ],
#                           [-1.0, 0.0, 0.0 ],
#                           [ 0.0, 0.0, 1.0 ]])    # 6軸計測モジュールから見た, 6軸センサの位置 

# FootZ Configuration
#        self.position = np.array( [ -56, 0, 0 ] ) # 6軸計測モジュールから見た, 6軸センサの姿勢
#        self.R = np.array([[ 1.0, 0.0, 0.0 ],
#                           [ 0.0, 1.0, 0.0 ],
#                           [ 0.0, 0.0, 1.0 ]])    # 6軸計測モジュールから見た, 6軸センサの位置 

        self.rawF_offset = np.zeros(3)
        self.rawT_offset = np.zeros(3)

        self.iscalibration = False

        self.calibbuf_F = []
        self.calibbuf_T = []

    def callback( self, msg ):
        """
        """
        print( "callback" )
        raw_F = np.array( [ msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z ] )   
        raw_T = np.array( [ msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z ] )

        if self.iscalibration:
            self.calibbuf_F.append( raw_F )
            self.calibbuf_T.append( raw_T )

        validrot_F = np.dot( self.R, (raw_F - self.rawF_offset)/gravitycoef )  # N to kgf
        validrot_T = np.dot( self.R, (raw_T - self.rawT_offset)*1000/gravitycoef )              # Nm to kgf mm

        validposrot_F = validrot_F
        validposrot_T = validrot_T + np.cross( self.position, validrot_F )

        publishdata = WrenchStamped()
        publishdata.header = msg.header
        publishdata.wrench.force.x = validposrot_F[0]
        publishdata.wrench.force.y = validposrot_F[1]
        publishdata.wrench.force.z = validposrot_F[2]
        publishdata.wrench.torque.x = validposrot_T[0]
        publishdata.wrench.torque.y = validposrot_T[1]
        publishdata.wrench.torque.z = validposrot_T[2]
        self.pub.publish(publishdata)

    def docalib( self, req ):
        """
        """
        if req.duration <= 0:
            return
        del self.calibbuf_F[:]
        del self.calibbuf_T[:]
        self.iscalibration = True
        time.sleep( req.duration )
        self.iscalibration = False

        self.rawF_offset = np.average( np.array( self.calibbuf_F ), axis = 0 )
        self.rawT_offset = np.average( np.array( self.calibbuf_T ), axis = 0 )

        return

if __name__ == "__main__":
    if len( sys.argv ) < 4:
        print( "usage: " + sys.argv[0] + " <input topic> <output topic> <service name>" )
        sys.exit(1)

    a = WrenchFilter( sys.argv[1],\
                      sys.argv[2],\
                      sys.argv[3] )
    rospy.spin()
