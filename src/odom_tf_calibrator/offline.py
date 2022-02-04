#!/usr/bin/env python

from __future__ import print_function


import os
import pickle
import random

from ff_tf.tf2d import TF2d
from odom_tf_calibrator.csv import CSV
from odom_tf_calibrator.optimizer import Optimizer
from odom_tf_calibrator.datapoint import DataPoint





def random_indices( data, sample_size ):
    """ returns a list of random indices that contains no duplicates and don't exceed the range of data """
    size = len( data[data.keys()[0]] )
    indices = list(range(size))
    for k in range(sample_size):
        new_index = random.randrange(k-1, size)
        indices[k], indices[new_index] = indices[new_index], indices[k]
    return indices[0:sample_size]
    


class Offline( object ):
    
    def __init__( self, path='' ):
        self.path = path
        self.verbose = False
        self.keyframes = None
        self.data = None
        #self.csv = CSV( path=self.path )
        self.read_keyframes()
        self.gt = [0.958, -0.695, -0.790, -0.958, 0.695, 2.352] # ground truth for our simulated thorvald
        
    def compute_error( self, x, errors ):
        """ computes the absolute error against the ground truth and appends it to the list 
        x: tuple of the parameter estimates
        errors: list of lists to store the absolute errors"""
        for i in range(6):
            errors[i].append( abs(x[i]-self.gt[i]) )
    
    def error_stats( self, errors ):
        result = []
        for i in range(6):
            s_err = sorted( errors[i] )
            size = len( s_err )
            mean = sum( s_err ) / size
            median = s_err[ int(size/2) ]
            var = sum((x - mean)**2 for x in s_err) / size
            std_dev = var ** 0.5
            result.append( {'mean': mean, 'median': median, 'var': var, 'std_dev': std_dev, 'min':s_err[0], 'max':s_err[-1]} )
        if True:
            for r in result:
                print( '  mean={}, median={}, var={}, sigma={}, min={}, max={}'.format(r['mean'], r['median'], r['var'], r['std_dev'], r['min'], r['max']) )
        #print( result )
        return result
        
    
    def run( self, data=None, iterations=3 ):
        if not data:
            data = self.create_data()
        opt = Optimizer()
        initial_guess = (0., 0., 0., 0., 0., 0.)
        keys = ('odometry+base_raw', 'odom_front_scan', 'odom_back_scan')
        for k in range(iterations-1):
            #print( 'iteration #{}'.format(k) )
            result = opt.optimize( data, initial_guess, keys )
            initial_guess = result.x
            opt.compute_weights( data, result.x )
        #print( 'iteration #{}'.format(iterations-1) )
        result = opt.optimize( data, initial_guess, keys )
        #self.result_to_csv( data, result, opt )
        return result
    
    def run_2( self, batch_size=24, inc=4 ):
        """ sliding window in data """
        data = self.create_data()
        size = len( data[data.keys()[0]] )
        start = 0
        stop = 0
        results = []
        while True:
            start = start + inc
            stop = start + batch_size
            if stop >= size:
                print( 'break condition met' )
                break
            subset = self.create_data_subset( data, range(start, stop) )
            results.append( self.run(subset) )
        return results
    
    def run_2_random( self, batch_size=24, num_runs=4 ):
        """ sliding window in data """
        data = self.create_data()
        results = []
        for i in range(num_runs):
            subset = self.create_random_subset( data, batch_size )
            results.append( self.run(subset) )
        return results
    
    def run_2_sim( self, batch_size=16, errors=([],[],[],[],[],[],[]) ):
        """ sliding window in data, ground truth used to compute errors """
        data = self.create_data()
        size = len( data[data.keys()[0]] )
        start = 0
        stop = 0
        inc = 5
        print( 'run_2_sim: batch_size={}, inc={}'.format(batch_size, inc) )
        while True:
            start = start + inc
            stop = start + batch_size
            if stop >= size:
                break
            if stop > size:
                print( 'break condition met' )
                break
            subset = self.create_data_subset( data, range(start, stop) )
            result = self.run( subset )
            x = result.x
            self.compute_error( x, errors )
        self.error_stats( errors )
        return errors
    
    def run_3( self ):
        """ sliding window in data """
        data = self.create_data()
        size = len( data[data.keys()[0]] )
        inc = 5
        start = 0
        stop = 0
        while True:
            stop = stop + inc
            if stop >= size:
                break
            if stop > 100:
                print( 'break condition met' )
                break
            subset = self.create_data_subset( data, range(start, stop) )
            self.run( subset )
    
    def result_to_csv( self, data, result, opt ):
        """ result (x,y,t)x2,  result error (ex,ey,et)x2, norm_error (ne)x2, data_length """
        x = result.x
        norm_errors = opt.norm_errors( data, result.x )
        size = len( data[data.keys()[0]] )
        values = (x[0], x[1], x[2],  x[3], x[4], x[5],
            x[0]-0.958, x[1]+0.695, x[2]+0.790,  x[3]+0.958, x[4]-0.695, x[5]-2.352,
            norm_errors['odom_front_scan'], norm_errors['odom_back_scan'],
            size)
        self.csv.write( values )
    
    def data_to_csv( self, data=None ):
        """ write the data to a csv file """
        if not data:
            data = self.create_data()
        keys = data.keys()
        size = len( data[keys[0]] )
        csv = CSV( filename='data.csv' )
        header = '#'
        for k in keys:
            header = '{} {}'.format( header, k )
        csv.write_raw( header )
        csv.write_raw( '\n' )
        print( header )
        for i in range(size):
            d = [ i ]
            for k in keys:
                d = d + list(data[k][i])
            csv.write( d )
        
    def read_keyframes( self ):
        print( 'reading files from \'{}\''.format(self.path) )
        keyframes = dict()
        all_files = os.listdir( self.path )
        pickle_files = [ name for name in all_files if name.endswith('.pickle') ]
        pickle_files = sorted(pickle_files)
        
        rn_dict = dict()
        for i in range(len(pickle_files)):
            name_index = pickle_files[i].rfind( '_' )
            sensor_name = pickle_files[i][0:name_index]
            sensor_rn = int(pickle_files[i][name_index+1:-7])
            if not sensor_name in keyframes:
                keyframes[sensor_name] = []
                rn_dict[sensor_name] = []
            filename = self.path + '/' + pickle_files[i]
            rn_dict[sensor_name].append( sensor_rn )
            #print( 'filename: \'{}\''.format(filename) )
            with open(filename, 'rb') as file_handle:
                keyframe = pickle.load(file_handle)
                keyframes[sensor_name].append( keyframe )
        print( 'sensors found: {}'.format(keyframes.keys()) )
        print( 'read a total of {} files'.format(len(pickle_files)) )
        
        self.check_rn( rn_dict )
        
        for key in keyframes:
            print( '{}: {} entries'.format(key, len(keyframes[key])) )
        self.keyframes = keyframes
    
    def check_rn( self, rn_dict ):
        if self.verbose:
            print( 'checking running number of keyframes' )
            for key in rn_dict:
                slist = sorted(rn_dict[key])
                for i in range(1, len(slist)):
                    if slist[i-1]+1 != slist[i]:
                        print( '  missmatch: {}, {} ({})'.format(slist[i-1], slist[i], key) )
    
    def create_data( self ):
        """ creates the data that we use for the optimization process. we collect the delta movement between keyframes and return it.
        the returned object is a dictionary with the odom topic names as keys. each topic has a list attached, that contains (dx,dy,dtheta)
        tuples, effectifly a 2d array, though not a numpy array. """
        data = dict()
        for key in self.keyframes:  # iterate over every odom topic
            odom = self.keyframes[key]      # odom keyframes for a single topic
            length = len(odom)              # number of keyframes (same number for each odom topic)
            deltas = []                     # list where we store the odometry deltas
            for i in range(1,length):       # create all odometry deltas for a single topic
                old_frame = TF2d.from_xyt( odom[i-1][0], odom[i-1][1], odom[i-1][2] )
                new_frame = TF2d.from_xyt( odom[i][0], odom[i][1], odom[i][2] )
                delta_tf = new_frame * old_frame.inverse()
                deltas.append( DataPoint.from_xyt(delta_tf.as_xyt()) )
            data[key] = deltas
        #self.debug_data( data )
        #return data
        return self.outlier_fix( data )
    
    def outlier_fix( self, data ):
        """ removes any keyframes where any of the translational movements exceeds 1.5m """
        size = len( data[data.keys()[0]] )
        num_outliers = 0
        outlier = False
        keys = data.keys()
        new_data = dict()
        for k in keys:
            new_data[k] = []
        for i in range(size):
            outlier = False
            for k in keys:
                d = data[k][i]
                if abs(d[0]) > 1.5  or  abs(d[1]) > 1.5:
                    outlier = True
                    break
            if outlier:
                num_outliers += 1
            else:
                for k in keys:
                    new_data[k].append( data[k][i] )
        print( 'removed {} keyframes from {} keyframes, new size={}'.format(num_outliers, size, size-num_outliers) )
        return new_data
    
    def create_data_subset( self, data, indices ):
        """ creates a subset of the data dictionary containing only keyframes identified by indices """
        if data == None:
            data = self.create_data()
        
        subset = dict()
        for sensor in data:
            subset[sensor] = []
            for i in indices:
                subset[sensor].append( data[sensor][i] )
        return subset
    
    def create_random_subset( self, data, size ):
        indices = random_indices( data, size )
        return self.create_data_subset( data, indices )
    
    def create_smart_subset( self, data, size ):
        """ we first pick half the size with random indices and then choose the rest to create variety """
        # todo ...
        

