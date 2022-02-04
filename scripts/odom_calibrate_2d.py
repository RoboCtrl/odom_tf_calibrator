#!/usr/bin/env python

from __future__ import print_function

from threading import Thread

from odom_tf_calibrator.calibration_node import CalibratorNode

import rospy


def keyframe_callback( node_obj ):
    """ callback function that is called  every time a new set of keyframes is created """
    num_keyframes = len( node_obj.keyframes[ node_obj.odom_list[0] ] )
    # we start a calibartion thread every 5 keyframes. there's just no point to do so every single keyframe
    if num_keyframes % 5 == 0 and num_keyframes <= node_obj.max_keyframes:
        new_thread = Thread( target=node_obj.optimize )
        new_thread.start()
        #self.optimize()
    if num_keyframes == node_obj.save_at:
        for sub in node_obj.subscribers:
            sub.unregister()
        node_obj.subscribers = []
        print( 'calibartion finished, continueing to re-publish scans and tf' )
    #    rospy.signal_shutdown( 'calibratioerror_statsn finished' )
    if node_obj.save_data and num_keyframes == node_obj.save_at:
        node_obj.save_data_async()


if __name__ == '__main__':
    node = CalibratorNode()
    node.keyframe_callback = keyframe_callback
    rospy.spin()


