import cv2 as cv
import numpy as np
import os
import glob

image_folder = 'saved_images'

def main():
    print("Finished")

def save_image(frames):
    framecount=0
    for i,frame in enumerate(frames):
        filename=None
        while filename is None or os.path.isfile(filename):
            filename=image_folder+"/cam"+str(i)+"_frame"+str(framecount)+".png"
            framecount+=1
        print("saving Image Frame. Filename="+str(filename))
        cv.imwrite(filename, frame)
    
def calibrateCamera():
    image_folder_calibrated = f'{image_folder}_c'
    # cam_images_folder_name = 'cam_1'

    # Defining the dimensions of checkerboard
    CHECKERBOARD = (6,9)
    #CHECKERBOARD = (5,6)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = [] 
    
    
    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None
    
    # Extracting path of individual image stored in a given directory
    images = glob.glob(f'./{image_folder}/*.png')
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        # ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)
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
        
        # cv.imshow('img',img)
        # cv.waitKey(0)
        
        new_frame_name = image_folder_calibrated + '/' + os.path.basename(fname)
        # print(new_frame_name)
        cv.imwrite(new_frame_name, img)


    # cv.destroyAllWindows()

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

if __name__ == "__main__":
    main()