import cv2 as cv
import numpy as np
from Singleton import Singleton
#from helpers import camera_pose_to_serializable, calculate_reprojection_errors, bundle_adjustment, triangulate_points

from KalmanFilter import KalmanFilter
from helpers import make_square,find_point_correspondance_and_object_points, locate_objects

#@Singleton
class Cameras:
    def __init__(self):
        self.cameraparams_dir="camera-params"
        self.cameras = [] #Camera(fps=90, resolution=Camera.RES_SMALL, gain=10, exposure=100)
        self.cameranames=[]
        self.camera_params=[]
        

        #Save Frontend variables in backend
        self.objectPoints=[]
        self.objectPoints_current=[]
        self.objectPointErrors_current=[]
        self.objects_current=[]
        self.filteredObjects=[]
        self.image_points_captured=[] #when starting capture image points are collected here

        self.last_frames=[] #latest frames

        self.is_capturing_points = False
        self.is_triangulating_points = False
        self.camera_poses = None
        self.is_locating_objects = False
        self.to_world_coords_matrix = None
        self.num_objects = None
        self.kalman_filter = None
        self.socketio = None

        global cameras_init
        cameras_init = True

    def set_socketio(self, socketio):
        self.socketio = socketio

    def set_num_objects(self, num_objects):
        self.num_objects = num_objects
    
    def edit_settings(self, exposure, gain):
        pass


    def readFrames(self):
        raise NotImplementedError
    

    def _camera_read(self):
        frames = self.readFrames() #array of 2d arrays with value 0-255 for each pixel
        self.filteredObjects=[]
        self.objectPoints=[]

        if len(frames) is not self.num_cameras: #not all frames captured
            return None

        for i in range(0, self.num_cameras):
            frames[i] = make_square(frames[i])
            frames[i] = np.rot90(frames[i], k=self.camera_params[i]["rotation"])
            #frames[i] = make_square(frames[i])
            frames[i] = cv.undistort(frames[i], self.get_camera_params(i)["intrinsic_matrix"], self.get_camera_params(i)["distortion_coef"])
            frames[i] = cv.GaussianBlur(frames[i],(9,9),0)
            kernel = np.array([[-2,-1,-1,-1,-2],
                               [-1,1,3,1,-1],
                               [-1,3,4,3,-1],
                               [-1,1,3,1,-1],
                               [-2,-1,-1,-1,-2]])
            frames[i] = cv.filter2D(frames[i], -1, kernel)
            frames[i] = cv.cvtColor(frames[i], cv.COLOR_RGB2BGR)

        if (self.is_capturing_points):
            image_points = []
            for i in range(0, self.num_cameras):
                frames[i], single_camera_image_points = self._find_dot(frames[i])
                image_points.append(single_camera_image_points)
                
            
            if (any(np.all(point[0] != [None,None]) for point in image_points)):
                if self.is_capturing_points and not self.is_triangulating_points:
                    self.socketio.emit("image-points", [x[0] for x in image_points])
                    self.image_points_captured.append([x[0] for x in image_points])
                elif self.is_triangulating_points:
                    errors, object_points, frames = find_point_correspondance_and_object_points(image_points, self.camera_poses, frames)

                    # convert to world coordinates
                    for i, object_point in enumerate(object_points):
                        #new_object_point = np.array([[-1,0,0],[0,-1,0],[0,0,1]]) @ object_point
                        new_object_point=object_point
                        new_object_point = np.concatenate((new_object_point, [1])) #prepare for multiplication with affine matrix
                        new_object_point = np.array(self.to_world_coords_matrix) @ new_object_point #apply affine transformation
                        #new_object_point = new_object_point[:3] / new_object_point[3]
                        #new_object_point[1], new_object_point[2] = new_object_point[2], new_object_point[1]
                        #object_points[i] = new_object_point

                        object_points[i] = new_object_point[:3]
                        

                    
                    objects = []
                    filtered_objects = []
                    if self.is_locating_objects:
                        objects = locate_objects(object_points, errors)
                        filtered_objects = self.kalman_filter.predict_location(objects)
                                                            
                        for filtered_object in filtered_objects:
                            filtered_object["vel"] = filtered_object["vel"].tolist()
                            filtered_object["pos"] = filtered_object["pos"].tolist()

                        #print("Filtered Points:")
                        #print(filtered_objects) #[{'pos': [-0.2687288522720337, -0.13937008380889893, 0.753404974937439], 'vel': [-0.04375557228922844, -0.1639920473098755, -0.023906374350190163], 'heading': np.float64(-0.19185114768976597), 'droneIndex': 1}]
                    
                    self.socketio.emit("object-points", {
                        "object_points": object_points.tolist(), 
                        "errors": errors.tolist(), 
                        "objects": [{k:(v.tolist() if isinstance(v, np.ndarray) else v) for (k,v) in object.items()} for object in objects], 
                        "filtered_objects": filtered_objects
                    })
                    # see App.tsx line 238
                    
                    self.objectPoints=object_points
                    self.objectPoints_current.append(object_points.tolist())
                    self.objectPointErrors_current.append(errors.tolist())
                    self.objects_current.append([{k:(v.tolist() if isinstance(v, np.ndarray) else v) for (k,v) in object.items()} for object in objects])
                    self.filteredObjects=filtered_objects
        
        return frames

    def get_frames(self):
        frames = self._camera_read()
        #frames = [add_white_border(frame, 5) for frame in frames]
        if frames is not None:
            return np.hstack(frames)
        else:
            return None

    def _find_dot(self, img):
        # img = cv.GaussianBlur(img,(5,5),0)
        grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        grey = cv.threshold(grey, 255*0.2, 255, cv.THRESH_BINARY)[1]
        contours,_ = cv.findContours(grey, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        img = cv.drawContours(img, contours, -1, (0,255,0), 1)

        image_points = []
        for contour in contours:
            moments = cv.moments(contour)
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                cv.putText(img, f'({center_x}, {center_y})', (center_x,center_y - 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (100,255,100), 1)
                cv.circle(img, (center_x,center_y), 1, (100,255,100), -1)
                image_points.append([center_x, center_y])

        if len(image_points) == 0:
            image_points = [[None, None]]

        return img, image_points

    def start_capturing_points(self):
        self.is_capturing_points = True

    def stop_capturing_points(self):
        self.is_capturing_points = False

    def start_trangulating_points(self, camera_poses):
        self.is_capturing_points = True
        self.is_triangulating_points = True
        self.camera_poses = camera_poses
        self.kalman_filter = KalmanFilter(self.num_objects)

    def stop_trangulating_points(self):
        self.is_capturing_points = False
        self.is_triangulating_points = False
        # self.camera_poses = None #Todo: commented out

    def start_locating_objects(self):
        self.is_locating_objects = True

    def stop_locating_objects(self):
        self.is_locating_objects = False
    
    def get_camera_params(self, camera_num):
        return {
            "intrinsic_matrix": np.array(self.camera_params[camera_num]["intrinsic_matrix"]),
            "distortion_coef": np.array(self.camera_params[camera_num]["distortion_coef"]),
            "rotation": self.camera_params[camera_num]["rotation"]
        }
    
    def set_camera_params(self, camera_num, intrinsic_matrix=None, distortion_coef=None):
        if intrinsic_matrix is not None:
            self.camera_params[camera_num]["intrinsic_matrix"] = intrinsic_matrix
        
        if distortion_coef is not None:
            self.camera_params[camera_num]["distortion_coef"] = distortion_coef


