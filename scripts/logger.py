#!/usr/bin/env python

from __future__ import print_function

import os
#import sys
import time
import pickle


from threading import Thread

from odom_tf_calibrator.calibration_node import CalibratorNode

import rospy



path = ''
skip = 2



def setup():
    global path
    date_string = time.strftime( '%Y-%m-%d__%X' )
    path = '/opt/shared/uol/data/sensor_calibration/kitti_viso2/' + date_string + '/'
    #path = '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/' + date_string + '/'
    print( 'saving data to \'{}\''.format(path) )
    if not os.path.exists( path ):
        os.makedirs( path )


def keyframe_callback( node_obj ):
    global skip
    kbuffer = []
    size = len( node_obj.keyframes[node_obj.keyframes.keys()[0]] )
    if skip > 0:
        if size % (skip+1):
            return
    for key in node_obj.keyframes:
        keyframe_list = node_obj.keyframes[key]
        keyframe = keyframe_list[size-1]
        fkey = key.replace( '/', '+' )
        filename = '{}{}_{}.pickle'.format( path, fkey, '{0:05d}'.format(size-1) )
        kbuffer.append( (filename, keyframe) )
    
    for filename,keyframe in kbuffer:
        print( 'saving {}'.format(filename) )
        with open(filename, 'wb') as file_handle:
            pickle.dump( keyframe, file_handle, pickle.HIGHEST_PROTOCOL )




if __name__ == '__main__':
    setup()
    node = CalibratorNode()
    node.keyframe_callback = keyframe_callback
    rospy.spin()


