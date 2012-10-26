import tempfile
import os

def create_temp_file(contents):
    '''
    create_temp_file creates a temporary file with a given string as its
    contents and returns a path to it.
    '''
    handle, path = tempfile.mkstemp()
    with os.fdopen(handle, 'w') as f:
        f.write(contents)
        f.close()
    return path

