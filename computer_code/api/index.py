from helpers import camera_pose_to_serializable, calculate_reprojection_errors, bundle_adjustment, Cameras, triangulate_points, essential_from_fundamental, motion_from_essential
from KalmanFilter import KalmanFilter

from flask import Flask, Response, request
import cv2 as cv
import numpy as np
import json
from scipy import linalg

from flask_socketio import SocketIO
import copy
import time
#import serial
import threading
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from flask_cors import CORS
import json


serialLock = threading.Lock()

#ser = serial.Serial("/dev/cu.usbserial-02X2K2GE", 1000000, write_timeout=1, )

app = Flask(__name__)
CORS(app, supports_credentials=True)
socketio = SocketIO(app, cors_allowed_origins='*')

cameras_init = False

num_objects = 2



@app.route("/api/camera-stream")
def camera_stream():
    cameras = Cameras.instance()
    cameras.set_socketio(socketio)
    #cameras.set_ser(ser)
    cameras.set_serialLock(serialLock)
    cameras.set_num_objects(num_objects)


    
    def gen(cameras):
        frequency = 150
        loop_interval = 1.0 / frequency
        last_run_time = 0
        i = 0

        while True:
            time_now = time.time()

            i = (i+1)%10
            if i == 0:
                socketio.emit("fps", {"fps": round(1/(time_now - last_run_time))})

            if time_now - last_run_time < loop_interval:
                time.sleep(last_run_time - time_now + loop_interval)
            last_run_time = time.time()
            frames = cameras.get_frames()
            if frames is None:
                continue
            jpeg_frame = cv.imencode('.jpg', frames)[1].tostring()

            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg_frame + b'\r\n')

    return Response(gen(cameras), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/api/trajectory-planning", methods=["POST"])
def trajectory_planning_api():
    data = json.loads(request.data)

    waypoint_groups = [] # grouped by continuious movement (no stopping)
    for waypoint in data["waypoints"]:
        stop_at_waypoint = waypoint[-1]
        if stop_at_waypoint:
            waypoint_groups.append([waypoint[:3*num_objects]])
        else:
            waypoint_groups[-1].append(waypoint[:3*num_objects])
    
    setpoints = []
    for i in range(0, len(waypoint_groups)-1):
        start_pos = waypoint_groups[i][0]
        end_pos = waypoint_groups[i+1][0]
        waypoints = waypoint_groups[i][1:]
        setpoints += plan_trajectory(start_pos, end_pos, waypoints, data["maxVel"], data["maxAccel"], data["maxJerk"], data["timestep"])

    return json.dumps({
        "setpoints": setpoints
    })

def plan_trajectory(start_pos, end_pos, waypoints, max_vel, max_accel, max_jerk, timestep):
    otg = Ruckig(3*num_objects, timestep, len(waypoints))  # DoFs, timestep, number of waypoints
    inp = InputParameter(3*num_objects)
    out = OutputParameter(3*num_objects, len(waypoints))

    inp.current_position = start_pos
    inp.current_velocity = [0,0,0]*num_objects
    inp.current_acceleration = [0,0,0]*num_objects

    inp.target_position = end_pos
    inp.target_velocity = [0,0,0]*num_objects
    inp.target_acceleration = [0,0,0]*num_objects

    inp.intermediate_positions = waypoints

    inp.max_velocity = max_vel*num_objects
    inp.max_acceleration = max_accel*num_objects
    inp.max_jerk = max_jerk*num_objects

    setpoints = []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
        setpoints.append(copy.copy(out.new_position))
        out.pass_to_input(inp)

    return setpoints


@socketio.on("acquire-floor")
def acquire_floor(data={}):
    print("### Function Acquire Floor ###")
    cameras = Cameras.instance()
    if "objectPoints" in data.keys():
        object_points = data["objectPoints"]
    else:
        object_points = cameras.objectPoints_current
        
    #print("object points input")
    #print(object_points)  # [[],[],[], [[x1,y1,z1],[x2,y2,z2]],....]
    object_points = np.array([item for sublist in object_points for item in sublist])

    print("!! object_points shape="+str(np.shape(object_points))) #223,3
    print("!! cameras.objectPoints_current shape="+str(np.shape(np.array([item for sublist in cameras.objectPoints_current for item in sublist]))))

    tmp_A = []
    tmp_b = []
    for i in range(len(object_points)):
        tmp_A.append([object_points[i,0], object_points[i,1], 1])
        tmp_b.append(object_points[i,2])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)

    #print("b="+str(b))
    #print("A="+str(A))
          

    fit, residual, rnk, s = linalg.lstsq(A, b)
    fit = fit.T[0]
    print("fit="+str(fit))

    plane_normal = np.array([[fit[0]], [fit[1]], [-1]])
    plane_normal = plane_normal / linalg.norm(plane_normal)
    up_normal = np.array([[0],[0],[1]], dtype=np.float32)
    print("up normal="+str(up_normal))

    plane = np.array([fit[0], fit[1], -1, fit[2]])


    # https://math.stackexchange.com/a/897677/1012327
    G = np.array([
        [np.dot(plane_normal.T,up_normal)[0][0], -linalg.norm(np.cross(plane_normal.T[0],up_normal.T[0])), 0],
        [linalg.norm(np.cross(plane_normal.T[0],up_normal.T[0])), np.dot(plane_normal.T,up_normal)[0][0], 0],
        [0, 0, 1]
    ])
    F = np.array([plane_normal.T[0], ((up_normal-np.dot(plane_normal.T,up_normal)[0][0]*plane_normal)/linalg.norm((up_normal-np.dot(plane_normal.T,up_normal)[0][0]*plane_normal))).T[0], np.cross(up_normal.T[0],plane_normal.T[0])]).T
    R = F @ G @ linalg.inv(F)

    R = R @ [[1,0,0],[0,-1,0],[0,0,1]] # i dont fucking know why

    cameras.to_world_coords_matrix = np.array(np.vstack((np.c_[R, [0,0,0]], [[0,0,0,1]])))

    print("to woorld coords matrix")
    print(cameras.to_world_coords_matrix)

    socketio.emit("to-world-coords-matrix", {"to_world_coords_matrix": cameras.to_world_coords_matrix.tolist()})


@socketio.on("set-origin")
def set_origin(data={}):
    print("### Function Set origin ###")
    cameras = Cameras.instance()
    print("data type "+str(type(data)))
    print("data keys "+str(data.keys()))
    for key in data.keys():
        print("")
        print("key "+str(key))
        print("np shape="+str(np.shape(data[key])))


    if "objectPoint" in data.keys():
        object_point = np.array(data["objectPoint"])
    else:
        object_point = np.array(cameras.objectPoints_current[0][0])  # diy frontent: [0.10317291036295734, -1.8458410174272768, -0.32667157689213683]

    print("!! object_point=")
    print(object_point)
    print("!! cameras.objectPoints_current[0][0]=")
    print(cameras.objectPoints_current[0][0])

    if "toWorldCoordsMatrix" in data.keys():
        to_world_coords_matrix = np.array(data["toWorldCoordsMatrix"])
    else:
        to_world_coords_matrix = cameras.to_world_coords_matrix
        

    
    print("!! to_world_coords_matrix=")
    print(to_world_coords_matrix)
    print("!! cameras.to_world_coords_matrix=")
    print(cameras.to_world_coords_matrix)


    transform_matrix = np.eye(4)

    object_point[1], object_point[2] = object_point[2], object_point[1] # i dont fucking know why
    transform_matrix[:3, 3] = -object_point

    to_world_coords_matrix = transform_matrix @ to_world_coords_matrix
    cameras.to_world_coords_matrix = to_world_coords_matrix

    print("")
    print("set origin result world coords matrix:")
    print(cameras.to_world_coords_matrix)

    socketio.emit("to-world-coords-matrix", {"to_world_coords_matrix": cameras.to_world_coords_matrix.tolist()})

'''
@socketio.on("update-camera-settings")
def change_camera_settings(data):
    cameras = Cameras.instance()
    
    cameras.edit_settings(data["exposure"], data["gain"])
'''

@socketio.on("capture-points")
def capture_points(data={}):
    start_or_stop = data["startOrStop"]
    cameras = Cameras.instance()

    if (start_or_stop == "start"):
        cameras.start_capturing_points()
        print("Cleared image points captured")
        cameras.image_points_captured=[] #clear collected points
        print("Starting points capture")
        return
    elif (start_or_stop == "stop"):
        cameras.stop_capturing_points()
        print("Stopping points capture")
        print("Collected "+str(np.shape(cameras.image_points_captured))+" points")

@socketio.on("calculate-camera-pose")
def calculate_camera_pose(data={}):
    print("### Function Calculate Camera Pose ###")
    cameras = Cameras.instance()
    if "cameraPoints" in data.keys():
        image_points = np.array(data["cameraPoints"])
    else:
        image_points = np.array(cameras.image_points_captured)


    
    print("!! image_points=")
    print(image_points)

    print("!! np.array(cameras.image_points)=")
    print(np.array(cameras.image_points_captured))

    image_points_t = image_points.transpose((1, 0, 2))

    camera_poses = [{
        "R": np.eye(3),
        "t": np.array([[0],[0],[0]], dtype=np.float32)
    }]
    for camera_i in range(0, cameras.num_cameras-1):
        camera1_image_points = image_points_t[camera_i]
        camera2_image_points = image_points_t[camera_i+1]
        not_none_indicies = np.where(np.all(camera1_image_points != None, axis=1) & np.all(camera2_image_points != None, axis=1))[0]
        camera1_image_points = np.take(camera1_image_points, not_none_indicies, axis=0).astype(np.float32)
        camera2_image_points = np.take(camera2_image_points, not_none_indicies, axis=0).astype(np.float32)

        F, _ = cv.findFundamentalMat(camera1_image_points, camera2_image_points, cv.FM_RANSAC, 1, 0.99999)
        E = essential_from_fundamental(F, cameras.get_camera_params(0)["intrinsic_matrix"], cameras.get_camera_params(1)["intrinsic_matrix"])
        possible_Rs, possible_ts = motion_from_essential(E)

        R = None
        t = None
        max_points_infront_of_camera = 0
        for i in range(0, 4):
            object_points = triangulate_points(np.hstack([np.expand_dims(camera1_image_points, axis=1), np.expand_dims(camera2_image_points, axis=1)]), np.concatenate([[camera_poses[-1]], [{"R": possible_Rs[i], "t": possible_ts[i]}]]))
            object_points_camera_coordinate_frame = np.array([possible_Rs[i].T @ object_point for object_point in object_points])

            points_infront_of_camera = np.sum(object_points[:,2] > 0) + np.sum(object_points_camera_coordinate_frame[:,2] > 0)

            if points_infront_of_camera > max_points_infront_of_camera:
                max_points_infront_of_camera = points_infront_of_camera
                R = possible_Rs[i]
                t = possible_ts[i]

        R = R @ camera_poses[-1]["R"]
        t = camera_poses[-1]["t"] + (camera_poses[-1]["R"] @ t)

        camera_poses.append({
            "R": R,
            "t": t
        })

    camera_poses = bundle_adjustment(image_points, camera_poses, socketio)

    object_points = triangulate_points(image_points, camera_poses)
    error = np.mean(calculate_reprojection_errors(image_points, object_points, camera_poses))

    print("Camera Pose=")
    print(camera_poses)

    socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})
    cameras.camera_poses=camera_poses

@socketio.on("locate-objects")
def start_or_stop_locating_objects(data={}):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]

    if (start_or_stop == "start"):
        cameras.start_locating_objects()
        print("Started locating objects")
        return
    elif (start_or_stop == "stop"):
        cameras.stop_locating_objects()
        print("Stopped locating objects")

@socketio.on("determine-scale")
def determine_scale(data={}):
    print("### Function Determin scale ###")
    cameras = Cameras.instance()
    if "objectPoints" in data.keys():
        object_points = data["objectPoints"]
    else:
        object_points = cameras.objectPoints_current

    try:
        print("!! object_points len=")
        print(len(object_points))
        print("!! cameras.objectPoints_current len=")
        print(len(cameras.objectPoints_current))
    except:
        pass
    if "cameraPoses" in data.keys():
        camera_poses = data["cameraPoses"]
    else:
        camera_poses=cameras.camera_poses

    if camera_poses is None:
        print("using default camera_poses")
        #cameras.camera_poses are set to None when triangulation is stopped by stop_trangulating_points() in helpers.py
        camera_poses=[{ "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0, 0, 0] }, { "R": [[-0.0008290000610233772, -0.7947131755287576, 0.6069845808584402], [0.7624444396180684, 0.3922492478955913, 0.5146056781855716], [-0.6470531579819294, 0.46321862674804054, 0.6055994671226776]], "t": [-2.6049886186449047, -2.173986915510569, 0.7303458563542193] }, { "R": [[-0.9985541623963866, -0.028079891357569067, -0.045837806036037466], [-0.043210651917521686, -0.08793122558361385, 0.9951888962042462], [-0.03197537054848707, 0.995730696156702, 0.0865907408997996]], "t": [0.8953888630067902, -3.4302652822708373, 3.70967106300893] }, { "R": [[-0.4499864100408215, 0.6855400696798954, -0.5723172578577878], [-0.7145273934510732, 0.10804105689305427, 0.6912146801345055], [0.5356891214002657, 0.7199735709654319, 0.4412201517663212]], "t": [2.50141072072536, -2.313616767292231, 1.8529907514099284] }] #default settings. from app.tsx


    try:
        print("!! camera_poses=")
        print(camera_poses)
        print("!! cameras.camera_poses=")
        print(cameras.camera_poses) 
    except:
        pass

    actual_distance = 0.15
    observed_distances = []

    for object_points_i in object_points:
        if len(object_points_i) != 2:
            continue

        object_points_i = np.array(object_points_i)

        observed_distances.append(np.sqrt(np.sum((object_points_i[0] - object_points_i[1])**2)))

    scale_factor = actual_distance/np.mean(observed_distances)
    for i in range(0, len(camera_poses)):
        camera_poses[i]["t"] = (np.array(camera_poses[i]["t"]) * scale_factor).tolist()

    print("finish determine scale. camera poses=")
    print(camera_poses)
    socketio.emit("camera-pose", {"error": None, "camera_poses": camera_poses})
    cameras.camera_poses=camera_poses


@socketio.on("triangulate-points")
def live_mocap(data={}):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]
    if "cameraPoses" in data.keys():
        camera_poses = data["cameraPoses"]
    else:
        camera_poses=cameras.camera_poses

    '''
    if camera_poses is None:
        print("using default camera_poses")
        #cameras.camera_poses are set to None when triangulation is stopped by stop_trangulating_points() in helpers.py
        camera_poses=[{ "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0, 0, 0] }, { "R": [[-0.0008290000610233772, -0.7947131755287576, 0.6069845808584402], [0.7624444396180684, 0.3922492478955913, 0.5146056781855716], [-0.6470531579819294, 0.46321862674804054, 0.6055994671226776]], "t": [-2.6049886186449047, -2.173986915510569, 0.7303458563542193] }, { "R": [[-0.9985541623963866, -0.028079891357569067, -0.045837806036037466], [-0.043210651917521686, -0.08793122558361385, 0.9951888962042462], [-0.03197537054848707, 0.995730696156702, 0.0865907408997996]], "t": [0.8953888630067902, -3.4302652822708373, 3.70967106300893] }, { "R": [[-0.4499864100408215, 0.6855400696798954, -0.5723172578577878], [-0.7145273934510732, 0.10804105689305427, 0.6912146801345055], [0.5356891214002657, 0.7199735709654319, 0.4412201517663212]], "t": [2.50141072072536, -2.313616767292231, 1.8529907514099284] }] #default settings. from app.tsx
    '''
    
    
    try:
        print("!! camera_poses=")
        print(camera_poses)
        print("!! cameras.camera_poses=")
        print(cameras.camera_poses)
    except:
        pass

    try:
        print("!! 1 cameras.to_world_coords_matrix")
        print(cameras.to_world_coords_matrix)
    except:
        pass

    if "toWorldCoordsMatrix" in data.keys(): 
        cameras.to_world_coords_matrix = data["toWorldCoordsMatrix"]
    if cameras.to_world_coords_matrix is None:
        print("using default to world coords matrix")
        cameras.to_world_coords_matrix=[[0.9941338485260931, 0.0986512964608827, -0.04433748889242502, 0.9938296704767513], [-0.0986512964608827, 0.659022672138982, -0.7456252673517598, 2.593331619023365], [0.04433748889242498, -0.7456252673517594, -0.6648888236128887, 2.9576262456228286], [0, 0, 0, 1]]  #default / starting value. taken from app.tsx

    try:
        print("!! 2 cameras.to_world_coords_matrix")
        print(cameras.to_world_coords_matrix) 
    except:
        pass
    

    if (start_or_stop == "start"):
        cameras.start_trangulating_points(camera_poses)
        cameras.objectPoints_current=[]
        print("Cleared objectPoints")
        print("Started triangulating points")
        return
    elif (start_or_stop == "stop"):
        cameras.stop_trangulating_points()
        print("Stopped triangulating points")
        print("Captured "+str(len(cameras.objectPoints_current))+" object Points")


if __name__ == '__main__':
    socketio.run(app, port=3001, debug=True, host='0.0.0.0')