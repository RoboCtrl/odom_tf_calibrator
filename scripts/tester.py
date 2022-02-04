#!/usr/bin/env python

from __future__ import print_function


import os
import pickle



class Tester( object ):
    def __init__( self ):
        pass




class Offline( object ):
    
    def __init__( self ):
        self.path = '/opt/shared/uol/data/sensor_calibration/sim/' + '2022-01-26__18:43:58'

    def read_data( self ):
        data = dict()
        all_files = os.listdir( self.path )
        pickle_files = [ name for name in all_files if name.endswith('.pickle') ]
        pickle_files = sorted(pickle_files)
        
        for i in range(len(pickle_files)):
            name_index = pickle_files[i].rfind( '_' )
            sensor_name = pickle_files[i][0:name_index]
            if not sensor_name in data:
                data[sensor_name] = []
                filename = self.path + '/' + pickle_files[i]
                with open(filename, 'w') as file_handle:
                    data[sensor_name].append(pickle.load(file_handle) )
        print( 'sensors found: '.format(data.keys()) )
        print( 'read a total of {} files'.format(len(pickle_files)) )
            



if __name__ == '__main__':
    pass