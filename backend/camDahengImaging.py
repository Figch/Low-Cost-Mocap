import gxipy as gx
import os
import json
import cv2 as cv
import copy
import numpy as np
from Singleton import Singleton
from cameras import Cameras


#@Singleton
class CamerasDahengImaging(Cameras):
    def __init__(self):
        super().__init__()
        
        # create a device manager
        self.device_manager = gx.DeviceManager()
        dev_num, dev_info_list = self.device_manager.update_device_list()
        if dev_num == 0:
            print("Number of enumerated devices is 0")
            return
        print("Found "+str(dev_num)+" cameras")
        print(dev_info_list)

        # open the first device
        for i in range(dev_num):
            cam = self.device_manager.open_device_by_index(i+1)
            
            # exit when the camera is a mono camera
            if cam.PixelColorFilter.is_implemented() is False:
                print("This sample does not support mono camera.")
                cam.close_device()
                return

            # set continuous acquisition
            cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
            
            print("Resolution:"+str(cam.WidthMax.get())+"x"+str(cam.HeightMax.get()))
            cam.Width.set(cam.WidthMax.get())
            cam.Height.set(cam.HeightMax.get())
            #cam.Width.set(800)
            #cam.Height.set(600)
            cam.DecimationHorizontal.set(2)
            cam.DecimationVertical.set(2)
            #cam.BinningHorizontal.set(2)
            #cam.BinningVertical.set(2)


            # set exposure
            cam.ExposureTime.set(40000.0) #40000.0

            # set gain
            cam.Gain.set(0.0)

            '''        
            # get param of improving image quality
            if cam.GammaParam.is_readable():
                gamma_value = cam.GammaParam.get()
                gamma_lut = gx.Utility.get_gamma_lut(gamma_value)
            else:
                gamma_lut = None
            if cam.ContrastParam.is_readable():
                contrast_value = cam.ContrastParam.get()
                contrast_lut = gx.Utility.get_contrast_lut(contrast_value)
            else:
                contrast_lut = None
            if cam.ColorCorrectionParam.is_readable():
                color_correction_param = cam.ColorCorrectionParam.get()
            else:
                color_correction_param = 0
            '''


            # set the acq buffer count
            cam.data_stream[0].set_acquisition_buffer_number(1)
            # start data acquisition
            cam.stream_on()

            self.cameras.append(cam)
            cameraname=dev_info_list[i]["device_id"].replace(' ','-').replace('(','_').replace(')','')
            print("Added Camera with name: "+str(cameraname))
            self.cameranames.append(cameraname)

            dirname = os.path.dirname(__file__)
            filename = os.path.join(dirname, self.cameraparams_dir+"/"+cameraname+".json")  

            if not os.path.isfile(filename):
                print("Calibration for camera not found:"+str(filename)+" . Using default.")
                filename = os.path.join(dirname, "default.json")  
            
            with open(filename) as f: 
                self.camera_params.append(json.load(f))

        self.num_cameras = dev_num #len(self.cameras.exposure)

    def readFrames(self):
        frames=[]
        for i in range(self.num_cameras):            
            # get raw image
            raw_image = self.cameras[i].data_stream[0].get_image()
            if raw_image is None:
                print("Getting image failed.")
                continue

            # get RGB image from raw image
            try:
                rgb_image = raw_image.convert("RGB")
            except:
                continue

            if rgb_image is None:
                continue

            # improve image quality
            #rgb_image.image_improvement(color_correction_param, contrast_lut, gamma_lut)

            # create numpy array with data from raw image
            numpy_image = rgb_image.get_numpy_array()
            if numpy_image is None:
                continue

            #numpy_image = cv.resize(numpy_image, (800,600), interpolation= cv.INTER_LINEAR) 

            pimg = cv.cvtColor(np.asarray(numpy_image),cv.COLOR_BGR2RGB)*1.0

            frames.append(pimg)

        self.last_frames=copy.deepcopy(frames)
            
        return frames