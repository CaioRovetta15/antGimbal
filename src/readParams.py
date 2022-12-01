import numpy as np
import time
import cv2
import cv2.aruco as aruco
import pandas as pd

def readParams( file, camera ) :

    # TODO: Resolve exception of missing the file
    data = pd.read_csv(file)    
        
    for index in range( 0, data.index.stop ) :
        row = data.iloc[index]
        if row['CAMERA'] == camera :
            # Read dist array
            dist = np.array([])
            for i in range(1, 6) : dist = np.append( dist, row[i] )
            dist = np.array( [dist] )
            # Read MTX matrix
            mtx = np.array(row[6:])
            mtx = mtx.reshape(3,3)
            return dist, np.array(mtx, dtype=float)

    return None, None

if __name__ == "__main__" :
    readParams('calibration_data.csv', 'PC Calvin')