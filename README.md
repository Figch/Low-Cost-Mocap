# Low Cost Mocap

### A general purpose motion capture system built from the ground up

This repository is based on [Low-Cost-Mocap](https://github.com/jyjblrd/Low-Cost-Mocap) by jyjblrd.
It has been reduced to the tracking ability, removing the drone flying controls, as well as fixing and simplifying the calibration procedure.

The tracked markers and objects are sent via osc.

## Hardware

Tested with two Daheng Imaging MER2-202-60GC-P GigE Cameras.

## Dependencies

Tested on Python 3.12.6. Requirements see backend/requirements.txt
When using Daheng Imaging Cameras, the SDK is needed: [https://va-imaging.com/collections/software-development-kit-daheng-imaging-cameras](https://va-imaging.com/collections/software-development-kit-daheng-imaging-cameras)

## Running the backend

Optional: Inside backend directory create a python venv and install the requirements.

Then run the backend with `python3 api/index.py`.

To specify the host ip and port for the api controls, use -i or --ip <ip-address> and -p or --port <port>. Default 0.0.0.0:3001.

OSC tracking data is sent to --osc-ip <ip-address> on port --osc-port <port>. Defaults to 127.0.0.1 port 3002.

## Running the simplified Frontend

The backend is modified to store all calibration data and points captured for calibration. Therefore a simplified frontend only needs to display the camera stream and trigger the api endpoints.

To run it, open [simpleFrontend/index.html](simpleFrontend/index.html).

## Running the original frontend

From the frontend directory Run `yarn install` to install node dependencies 

Then run `yarn run dev` to start the webserver. You will be given a url view the frontend interface.