
import time



class CSV( object ):
    def __init__( self, filename=None, path=None ):
        if path == None:
            self.path = ''
        else:
            self.path = path
        if not filename:
            self.filename = time.strftime( '%Y-%m-%d__%X' ) + '.csv'
        else:
            self.filename = filename
        self.csv_file = None
    
    def write_raw( self, string ):
        if not self.csv_file:
            full_filename = self.path + self.filename
            self.csv_file = open( full_filename, 'w' )
        self.csv_file.write( string )
    
    def write( self, data ):
        """ takes data (a list) and writes it to the CSV file as comma-separated values and finishes the line with a line break """
        if not self.csv_file:
            full_filename = self.path + self.filename
            self.csv_file = open( full_filename, 'w' )
        string = str(data)
        self.csv_file.write( string[1:-1] )
        self.csv_file.write( '\n' )
    
    def close( self ):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None

