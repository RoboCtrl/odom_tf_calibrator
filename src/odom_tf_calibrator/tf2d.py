#!/usr/bin/env python
# -*- coding: utf-8 -*- 
# note: the encoding is needed for the TF2d.as_string method, which uses the degree symbol in its output string


from math import sin, cos, pi
import UserList
import numpy as np

try:
    from tf.transformations import euler_from_quaternion
except:
    #print( "Warning: unable to import ROS module 'tf.transformations'. TF2d.from_pos_quat method is not useable" )
    pass



class Rotation( UserList.UserList ):
    """ This class models a 2D rotation. The rotation itself is stored in 'data' as a numpy (rotation) matrix.
    The Rotation class is derived from UserList, allowing easy access to the matrix elements """
    
    def __init__( self, theta = 0., data = None ):
        """ if 'data' is provided (assumed to be a numpy matrix or compatible), it is used for the rotation matrix.
        if instead 'theta' is provided, we use it (as rad angle) to construct the rotation matrix. """
        if data is None:
            self.data = np.dot( np.identity(2), [  [cos(theta), -sin(theta)],  [sin(theta), cos(theta)]  ] )
        else:
            self.data = data
    
    def __str__( self ):
        return str(self.data)
    
    @classmethod
    def from_rad( cls, rad ):
        """ constructor that takes the rotation angle as radians (this is also the default behaviour of the Rotation class, and just
        added here for completeness) """
        return cls( rad )
    
    @classmethod
    def from_deg( cls, deg ):
        """ an alias for 'from_degree' """
        return cls.from_degree( deg )
    
    @classmethod
    def from_degree( cls, deg ):
        """ same as from_rad, but expectes the argument to be in degrees instead """
        return cls( deg / 180. * pi )
    
    @classmethod
    def from_rot_matrix( cls, mat_r ):
        """ constructor that takes a 2D rotation matrix """
        r = cls()
        r.data = np.dot( np.identity(2), mat_r ) # this should work, even in mat_r is not a numpy array, and always returns a numpy array
        return r
    
    def inverse( self ):
        """ returns a new instance with the rotation angle inverted """
        return self.transpose()
    
    def transpose( self ):
        """ creates a new instance with the diagonal elements swapt. this transposes the matrix, and in case of
        a rotation matrix also creates the inverse of the original matrix. """
        new_r = self.__class__()
        new_r.data = [ [self.data[0][0], self.data[1][0]], [self.data[0][1], self.data[1][1]] ]
        return new_r
    
    def get_angle( self ):
        """ returns the angle of the rotation (in rad) """
        return np.arctan2( self.data[1][0], self.data[0][0] )
    
    def get_angle_deg( self ):
        """ returns the angle of the rotation (in degree) """
        rad = self.get_angle()
        return rad * 180.0 / pi
    

class TF2d(object):
    def __init__( self, r = np.identity(2), t = np.zeros((2)) ):
        self.r = Rotation( data = r )
        self.t = np.array( t )
        if len(self.t.shape) > 1 and self.t.shape[0] == 2 and self.t.shape[1] == 2:
            raise NameError( 't is a 2x2 matrix, but should be a vector' )

    @classmethod
    def from_deg_and_point( cls, deg, point ):
        """ create and return a new instance with the provided rotation (in degree) and translation """
        return cls( Rotation.from_deg(deg), point )
    
    @classmethod
    def from_xyt( cls, x, y, theta):
        """ create and return a new instance with the provided rotation (in degree) and translation """
        return cls( Rotation(theta), [x, y] )
    
    @classmethod
    def from_sdl_pose( cls, pose_str ):
        items = pose_str.split()
        return cls( Rotation.from_rad(items[5], [items[0], items[1]]) )
    
    def __str__(self):
        return 'TF2d: R=' + str(self.r)+', t='+str(self.t)
        
    def __mul__(self, other):
        """ standard matrix multiplication """
        #new_t = np.array(other.t) + np.dot( other.r, self.t )
        #new_r = Rotation( data = np.dot( self.r, other.r ) )
        new_t = np.array(other.t) + np.matmul( other.r, self.t )
        new_r = Rotation( data = np.matmul( self.r, other.r ) )
        return TF2d( new_r, new_t )
    
    def __add__( self, other ):
        """ rotation matrices are multiplied, translations are added (without applying multiplication """
        new_t = np.array(other.t) + self.t
        #new_r = Rotation( data = np.dot( self.r, other.r ) )
        new_r = Rotation( data = np.matmul( self.r, other.r ) )
        return TF2d( new_r, new_t )
    
    def __getitem__(self, index):
        if index == 0:
            return self.r
        if index == 1:
            return self.t
        raise NameError('index ' + str(index) + ' is out of bounds')
    
    def as_string( self ):
        """ as string is similar to the __str__ method, but returns a more human-readable format """
        angle_deg = self.r.get_angle() * 180. / pi
        return 'TF(R={:5.3f}°, t=({:1.2f}, {:1.2f}))'.format( angle_deg, self.t[0], self.t[1] )
        
    def as_point( self ):
        return (self.t[0], self.t[1])
    
    def as_matrix( self ):
        m = np.matrix( [  [self.r[0][0], self.r[0][1], self.t[0]],  [self.r[1][0], self.r[1][1], self.t[1]],  [0.,0.,1.]  ] )
        return m
    
    def as_sdl_pose( self, z=0 ):
        """ returns a string that defines a pose in sdl, not including the pose tag itself. z is set to 0 by default, but
        can be adjusted via the 'z' argument.
        example return value: '1.5 -0.5 0  0 0 3.141529' """
        return '{} {} {}  0 0 {}'.format(self.t[0], self.t[1], z, self.r.get_angle() )
    
    def as_degree_and_point( self ):
        return ( self.r.get_angle()/pi*180, (self.t[0], self.t[1]) )
    
    def as_xyt( self ):
        """ returns x, y, and theta (rad) """
        return self.t[0], self.t[1], self.r.get_angle()
    
    def from_pos_quat( self, pos, quat ):
        angles = euler_from_quaternion( quat )
        self.t = np.array( (pos[0], pos[1]) )
        self.r = Rotation( angles[2] )
        #print( 'from_pos_quat:', pos, quat, str(self) )
        return self
    
    def step( self, factor ):
        """ returns a new instance that represents a motion that is multiplied by 'factor'. e.g. if factor is 0.5,
        the new tf2d instance rotates by half the original angle, and translates by half the distance """
        angle = self.r.get_angle()
        r = Rotation.from_rad( angle * factor )
        return self.__class__(r, self.t * factor )
    
    def transform( self, pointcloud ):
        """ returns a new pointcloud that has been transformed by self. the original pointcloud is not modified. """
        new_pc = pointcloud.rotate_matrix( self.r )
        return new_pc.apply_translate( self.t )
    
    def transform_point( self, point ):
        """ returns a new point (numpy.array) that represents the provided point, transformed by self. the original point is not modified """
        return np.matmul(self.r, np.array(point)) + self.t
    
    def transform_points( self, point_list ):
        """ returns a new list with points that are transformed based on self. neither the original list nor its content are modified """
        return [ np.matmul(self.r, p) + self.t for p in point_list ]
    
    def apply_transform( self, pointcloud ):
        """ applies the transformation to the provided pointcloud. note, that all apply_* methods modify the target, instead of creating a new instance. """
        pointcloud.apply_rotate_matrix( self.r )
        return pointcloud.apply_translate( self.t )
    
    def inverse( self ):
        """ returns a new instance that represents the inverse matrix """
        new_r = self.r.inverse()
        #new_t = np.dot( new_r.transpose(), (self.t * -1.))
        new_t = np.dot( new_r, (self.t * -1.))
        return TF2d( new_r, new_t )
    
    def self_check( self ):
        """ returns true, if the deteminate is close to 1.0, false otherwise """
        det = np.linalg.det( self.r )
        print( 'det R=' + str(det) )
        return abs( 1.0 - det ) < 0.0000001
    
    def is_similar_to( self, other, tolerance = 0.0000001 ):
        """ computes the sum of absolute deltas for each matrix entry. returns true, if the sum is less then 'tolerance', false otherwise """
        error = 0.0
        error += abs( self.r[0][0] - other.r[0][0] )
        error += abs( self.r[0][1] - other.r[0][1] )
        error += abs( self.r[1][0] - other.r[1][0] )
        error += abs( self.r[1][1] - other.r[1][1] )
        error += abs( self.t[0] - other.t[0] )
        error += abs( self.t[1] - other.t[1] )
        return error < tolerance
    
    def compare_to( self, other ):
        """ alias for self.is_similar_to """
        return self.is_similar_to( other )
    

