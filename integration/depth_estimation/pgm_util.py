import re
import subprocess

import numpy as np
import matplotlib.image as mpimg

# Stole this function from
# http://stackoverflow.com/questions/7368739/numpy-and-16-bit-pgm/7369986#7369986
def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.
    Format specification: http://netpbm.sourceforge.net/doc/pgm.html
    """
    with open(filename, 'rb') as f:
        buffer_ = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer_).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer_,
                         dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                         count=int(width)*int(height),
                         offset=len(header)
                         ).reshape((int(height), int(width)))

def write_pgm(image, filename):
    """ Write grayscale image in PGM format to file.
    """

    height, width = image.shape
    maxval = image.max()
    with open(filename, 'wb') as f:
        f.write('P5 {} {} {}\n'.format(width, height, maxval))
        # not sure if next line works universally, but seems to work on my mac
        image.tofile(f)

def CR2_to_pgm(filename):
    """ Use dcraw command line tool to convert Canon RAW format file (.CR2 extension) to PGM file.
    """
    cmd = 'dcraw -D -4 -j -t 0 '+filename
    subprocess.call(cmd, shell=True)

def jpg_to_pgm(filename, Rfile, Gfile, Bfile):
    """ Use matplotlib to convert color jpg file into three PGM files, one for each color channel.
    """
    img = mpimg.imread(filename)
    write_pgm(img[:,:,0], Rfile)
    write_pgm(img[:,:,1], Gfile)
    write_pgm(img[:,:,2], Bfile)