#!/usr/bin/env python

from __future__ import print_function

import UserList



class DataPoint( UserList.UserList ):
    """ represents a data point (delta movement) with an optional weight attribute. the object can be treated like a tuple (i.e. a list)
    or with the property names: x, y, and t. note, that 'weight' has no index and is only available as property. """
    def __init__( self, x, y, t, weight=1.0 ):
        super(DataPoint, self).__init__( (x,y,t) )
        self.weight = weight
    
    def __str__( self ):
        return '{}, {}, {} weight={}'.format( self[0], self[1], self[2], self.weight )
    
    @classmethod
    def from_xyt( cls, xyt ):
        return cls( xyt[0], xyt[1], xyt[2] )
        
    def get_x( self ):
        return self[0]
    
    def get_y( self ):
        return self[1]
    
    def get_t( self ):
        return self[2]
    
    def set_x( self, x ):
        self[0] = x
    
    def set_y( self, y ):
        self[1] = y
    
    def set_t( self, t ):
        self[2] = t
    
    def del_value( self ):
        pass
    
    x = property( get_x, set_x, del_value )
    y = property( get_y, set_y, del_value )
    t = property( get_t, set_t, del_value )



if __name__ == '__main__':
    print( 'standalone operation not supported at the moment' )
