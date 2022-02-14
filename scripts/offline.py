#!/usr/bin/env python


""" This node is for offline computing on batch data, mostly to test different parameters and algorithm variants quickly.

The data is expected to be in a folder as pickled keyframes for each sensor.
With our Thorvald robot that means three files for each keyframe index: odom, laser_front & laser_back.
"""


from __future__ import print_function


import time

from odom_tf_calibrator.csv import CSV
from odom_tf_calibrator.offline_processor import Offline



class TestRuns( object ):
    def __init__( self ):
        pass
    
    def write_result( self, csv, result, source_id ):
        for r in result:
            x = r.x
            csv.write_raw( '{} {} {} {} {} {} {} {}\n'.format(batch_size, x[0], x[1], x[2], x[3], x[4], x[5], source_id) )
    
    def run( self, sources, batches, window_shift=8, csv=None ):
        for batch_size in batches:
            time_start = time.time()
            source_id = 0
            print( 'running batch_size={}'.format(batch_size) )
            for path in sources:
                offline = Offline( path )
                result = offline.run_2( batch_size=batch_size, inc=window_shift )
                if csv:
                    self.write_result( csv, result,source_id)
                source_id += 1
            time_spend = time.time() - time_start
            print( '  time spent: {:.1%}s'.format(time_spend) )
    
    def thorvald_25_farm( self ):
        path = '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/odom_farm/'
        csv = CSV( 'odom_farm_16_256.csv', '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/' )

        self.run( [path], range(16, 257, 16), csv=csv )
        


def sim_batch_no_groundtruth():
    """ batch test on data from gazebo runs. merging data from two or more simulations """
    sources = (
        #'/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__21:20:10' + '/',
        '/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__22:53:57' + '/',
        #'/opt/shared/uol/data/sensor_calibration/sim/' + '2022-02-06__16:09:32' + '/',
        )
    csv = CSV( '2022-01-26__22:53:57__16_256.csv', '/opt/shared/uol/data/sensor_calibration/sim/' )
    #csv = CSV( '2022-02-06__16:09:32__16_256.csv', '/opt/shared/uol/data/sensor_calibration/sim/' )
    for batch_size in range(16, 257, 16):
        print( '== batch size: {} =='.format(batch_size) )
        for path in sources:
            offline = Offline( path )
            result = offline.run_2( batch_size=batch_size, inc=8 )
            for r in result:
                x = r.x
                csv.write_raw( '{} {} {} {} {} {} {}\n'.format(batch_size, x[0], x[1], x[2], x[3], x[4], x[5]) )
    

def sim_batch():
    """ batch test on data from gazebo runs. merging data from two or more simulations """
    sources = (
        #'/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__21:20:10' + '/',
        #'/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__22:53:57' + '/'
        '/opt/shared/uol/data/sensor_calibration/sim/' + '2022-02-06__16:09:32' + '/'
        )
    for batch_size in (16, 24, 32, 48, 64, 96, 128):
        print( '== batch size: {} =='.format(batch_size) )
        errors_combined = [ [], [], [],   [], [], [] ] # the first three lists are for the front scanner, the last three for the back scanner
        for path in sources:
            offline = Offline( path )
            errors = offline.run_2_sim( batch_size=batch_size )
        
        for i in range(len(errors)):
            errors_combined.append( errors[i] )
        
    print( 'combined error stats:' )
    error_stats = offline.error_stats( errors_combined )
    return  error_stats


def thorvald_025():
    path = '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/odom_farm/'
    all_results = []
    csv = CSV( 'odom_farm_16_256_no_outlier_removal.csv', '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/' )
    num_opt_iterations = 1
    for batch_size in range(16, 257, 16):
        print( 'batch_size={}'.format(batch_size) )
        offline = Offline( path )
        result = offline.run_2( batch_size=batch_size, inc=8, iterations=num_opt_iterations )
        all_results.append( (batch_size, result) )
    print( '--------' )
    for entry in all_results:
        batch = entry[1]
        print( 'batch_size={} ({})'.format( entry[0], len(batch) ) )
        for result in batch:
            print( 'x={}'.format(result.x) )
    print( '------------------------------' )
    for entry in all_results:
        batch_size = entry[0]
        batch = entry[1]
        for result in batch:
            x = result.x
            print( '{} {} {} {} {} {} {}'.format(batch_size, x[0], x[1], x[2], x[3], x[4], x[5]) )
            csv.write_raw( '{} {} {} {} {} {} {}\n'.format(batch_size, x[0], x[1], x[2], x[3], x[4], x[5]) )
    print( '------------------------------' )
        
def thorvald_025_random_subsets():
    path = '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/odom_farm/'
    all_results = []
    csv = CSV( 'odom_farm_random_16_256.csv', '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/' )
#    for batch_size in (8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128 ):
    for batch_size in range(16, 257, 16):
#    for batch_size in (8, 16, 24, 32, 40, 48, 56, 64):
        print( 'batch_size={}'.format(batch_size) )
        offline = Offline( path )
        result = offline.run_2_random( batch_size=batch_size, num_runs=64 )
        all_results.append( (batch_size, result) )
#    print( '--------' )
#    for entry in all_results:
#        batch = entry[1]
#        print( 'batch_size={} ({})'.format( entry[0], len(batch) ) )
#        for result in batch:
#            print( 'x={}'.format(result.x) )
    print( '------------------------------' )
    for entry in all_results:
        batch_size = entry[0]
        batch = entry[1]
        for result in batch:
            x = result.x
            print( '{} {} {} {} {} {} {}'.format(batch_size, x[0], x[1], x[2], x[3], x[4], x[5]) )
            csv.write_raw( '{} {} {} {} {} {} {}\n'.format(batch_size, x[0], x[1], x[2], x[3], x[4], x[5]) )
    print( '------------------------------' )



def kitti_test():
    #path = '/opt/shared/uol/data/sensor_calibration/kitti_viso2/2022-02-13__19:38:41'
    path = '/opt/shared/uol/data/sensor_calibration/kitti_viso2/2022-02-13__20:59:55/'
    offline = Offline( path )
    result = offline.run(keys=('mono_odometer_gray_left+odometry', 'mono_odometer_gray_right+odometry'))
    print( result )


if __name__ == '__main__':
    
    #kitti_test()
    #sim_batch_no_groundtruth()
    #sim_batch()
    #thorvald_025_random_subsets()
    #thorvald_025()
    testrun = TestRuns()
    testrun.thorvald_25_farm()
    
    
    if False:
        data_folder = '/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__21:20:10' + '/'
        data_folder_2 = '/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__22:53:57' + '/'
        #csv = CSV( path='', filename='combined_sim_error_stats.csv' )
        #csv.write_raw( '# num_keyframes, ( mean,  median,  var,  sigma,  min,  max ) x 6 \n' )
        for batch_size in (16, 24, 32, 48, 64, 96, 128):
            print( '== batch size: {} =='.format(batch_size) )
            offline = Offline( data_folder )
            errors = offline.run_2_sim( batch_size=batch_size )
            
            offline2 = Offline( data_folder_2 )
            errors2 = offline2.run_2_sim( batch_size=batch_size )
            
            errors_combined = []
            for i in range(len(errors)):
                errors_combined.append( errors[i] + errors2[i] )
            
            print( 'combined error stats:' )
            error_stats = offline2.error_stats( errors_combined )
            #line = [batch_size] + error_stats[0] + error_stats[1] + error_stats[2] + error_stats[3] + error_stats[4] + error_stats[5]
            #csv.write( line )
        #csv.close()

