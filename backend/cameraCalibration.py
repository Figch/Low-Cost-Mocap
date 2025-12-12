import cv2 as cv
import numpy as np
import os
import glob
import json
from datetime import datetime
import argparse


image_folder = 'saved_images'
image_folder_calibrated = 'saved_images/calibration'
calibrationDirectory = 'camera-params'
preview = False

def main():
    parser = argparse.ArgumentParser(description='')
    
    parser.add_argument('prefix', type=str, help='prefix of the image file. Name is followed by underscore and a number .png')
    parser.add_argument('-o', '--output', type=str, nargs='?',  help='output config filename. If not specified, using <prefix>.json')
    args = parser.parse_args()

    if args.output is not None:
        calibrateCamera(args.prefix,args.output)
    else:
        calibrateCamera(args.prefix)
    print("Finished")

def save_image(frames,cameranames=None):
    framecount=0
    for i,frame in enumerate(frames):
        filename=None
        current_cameraname="cam"+str(i)
        if cameranames is not None:
            current_cameraname=cameranames[i]
            
        while filename is None or os.path.isfile(filename):
            filename=image_folder+"/"+current_cameraname+"_"+str(framecount)+".png"
            framecount+=1
        print("saving Image Frame. Filename="+str(filename))
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        cv.imwrite(filename, frame)
    
def calibrateCamera(camName,calibrationFilename=None):
    #See https://github.com/jyjblrd/Low-Cost-Mocap/discussions/11
    if calibrationFilename is None:
        calibrationFilename=camName+".json"
    calibrationFilename=calibrationDirectory+"/"+calibrationFilename

    # Defining the dimensions of checkerboard
    
    #CHECKERBOARD = (7,10)
    #CHECKERBOARD_SQUARESIZE=0.025
    
    CHECKERBOARD = (6,9)
    CHECKERBOARD_SQUARESIZE=0.04
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = [] 
    
    
    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp*=CHECKERBOARD_SQUARESIZE
    prev_img_shape = None
    
    # Extracting path of individual image stored in a given directory
    #images = glob.glob(f'./{image_folder}/{camName}*.png')
    images = glob.glob(f'./{image_folder}/{camName}*.jpg')


    if len(images)<=0:
        print('No images found for calibration')
        exit()

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        
        _ , gray = cv.threshold(gray, 180,255, cv.THRESH_BINARY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        #ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)
        ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE)
        
        
        
        
        """
        If desired number of corner are detected,
        we refine the pixel coordinates and display 
        them on the images of checker board
        """
        if ret == True:
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            
            imgpoints.append(corners2)
    
            # Draw and display the corners
            img = cv.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        else:
            print("Corner detection failed")
        
        if preview:
            cv.imshow('img',img)
            cv.waitKey(0)
        
        new_frame_name = image_folder_calibrated + '/' + os.path.basename(fname)
        print(new_frame_name)
        os.makedirs(os.path.dirname(new_frame_name), exist_ok=True)
        cv.imwrite(new_frame_name, img)


    cv.destroyAllWindows()

    print("len(imgpoints)="+str(len(imgpoints)))
    print("len(objpoints)="+str(len(objpoints)))
    if len(imgpoints)<=0:
        print("Calibration Pattern not found")
        exit()


    h,w = img.shape[:2]

    """
    Performing camera calibration by 
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the 
    detected corners (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)

    output_config={"intrinsic_matrix":mtx.tolist(),"distortion_coef":dist.tolist(),"rotation":0,"created_at":datetime.now().strftime("%d/%m/%Y %H:%M:%S")}

    
    os.makedirs(os.path.dirname(calibrationFilename), exist_ok=True)
    with open(calibrationFilename, "w") as outfile: 
        json.dump(output_config, outfile, indent=4)
        print("Config saved as "+str(calibrationFilename))


if __name__ == "__main__":
    main()