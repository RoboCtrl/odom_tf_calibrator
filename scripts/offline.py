#!/usr/bin/env python


""" This node is for offline computing on batch data, mostly to test different parameters and algorithm variants quickly.

The data is expected to be in a folder as pickled keyframes for each sensor.
With our Thorvald robot that means three files for each keyframe index: odom, laser_front & laser_back.
"""


from __future__ import print_function


from odom_tf_calibrator.csv import CSV
from odom_tf_calibrator.offline import Offline





def sim_batch():
    data_folder = '/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__21:20:10' + '/'
    data_folder_2 = '/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__22:53:57' + '/'
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


def thorvald_025():
    path = '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/odom_farm/'
    all_results = []
    csv = CSV( 'odom_farm_16_256.csv', '/opt/shared/uol/data/sensor_calibration/thorvald_025/riseholme/' )
    for batch_size in range(16, 257, 16):
#    for batch_size in (8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128 ):
#    for batch_size in (128, ):
        print( 'batch_size={}'.format(batch_size) )
        offline = Offline( path )
        result = offline.run_2( batch_size=batch_size, inc=8 )
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
        



if __name__ == '__main__':
    
    #sim_batch()
    thorvald_025_random_subsets()
    thorvald_025()
    
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

