#!/usr/bin/env python3
########################################################################
# Filename    : __camera.py
# Description : thread videocapture via OpenCV
# Author      : Man'O'AR
# modification: 14/08/2021
########################################################################

import logging
from threading import Thread, Lock
import cv2 

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Camera( object ):
    
    def __init__( self, device_address = "/dev/video0", sensor_id =0, video_resolution = (960 , 540) ):

        super().__init__()

        self._capture = None
        self._device_address = device_address
        self._sensor_id = sensor_id
        self._output_resolution = video_resolution

        self._IsOpenCVEnabled = True

        self._gst_pipeline = None

        self.ret = False 
        self.frame = None 

        self._frame = None

        self._lock = Lock()
        self._thread_stopped = True
        self._thread_ = None

        self._isJetson = False


    @property
    def hardware_resolution(self): 
        return ( 1280, 720 )

            
    def _run( self ):
        
        while not self._thread_stopped:

            try:

                with self._lock:
                    
                    self._read_from_cv()

            except Exception as ex:
                logging.warning( f"an error has occured in videocapture : {ex}" )
                break 
    


    def _read_from_cv(self): 

        if self.ret:
                        
            self.ret, self.frame = self._capture.read()

            _,self._frame = cv2.imencode('.jpg', self.frame)#self._bridge.cv2_to_imgmsg( self.frame, encoding="bgr8" )


    def nv_pipeline(
        self,
        sensor=0,
        capture_width=1280,
        capture_height=720,
        display_width=320,
        display_height=240,
        framerate=30,
        flip_method=0,
    ):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "videoconvert ! "
            "videoscale ! "
            "video/x-raw, width=(int)%d, height=(int)%d ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
        )
    ), cv2.CAP_GSTREAMER


    def opencv_pipeline(
        self,
        sensor="/dev/video0",
        capture_width=1280,
        capture_height=720,
        display_width=320,
        display_height=240,
        framerate=30,
        flip_method=0,
    ):
        return (
        "v4l2src device=%s "
        "! video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 "
        "! videoflip method=%d "
        "! videoconvert "
        "! videoscale "
        "! video/x-raw, width=(int)%d, height=(int)%d "
        "! videoconvert "
        "! video/x-raw, format=(string)BGRx "
        "! videoconvert "
        #"! frei0r-filter-cartoon " #cartoonify
        #"! videoscale "
        "! video/x-raw, format=(string)BGR "
        "! appsink "
        % (
            sensor,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
        ), cv2.CAP_GSTREAMER
    

    def h264_pipeline(
        self,
        sensor="/dev/video0",
        width=640,
        height=480,
        framerate=30,
        flip_method=0,
        address="127.0.0.1"
    ):
        str_pipeline = (
            f"v4l2src device={sensor} "
            "! videoconvert "
            "! videoscale "
            f"! video/x-raw, width=(int){width}, height=(int){height}, framerate=(fraction){framerate}/1 "
            f"! videoflip method={flip_method} "
            #"! videoconvert "
            #"! frei0r-filter-cartoon "
            "! x264enc speed-preset=ultrafast tune=zerolatency " #byte-stream=true bitrate=3000 threads=1
            #"! h264parse config-interval=1 "
            "! rtph264pay "
            f"! udpsink host={address} port=3000 sync=false "
        )

        return str_pipeline
    

    def enable( self, peer_address = "127.0.0.1", OpenCV_render = True ):
        
        if self._capture is not None: 
            self.disable()

        self._IsOpenCVEnabled = OpenCV_render

        if OpenCV_render is True: 
            
            pipeline, apiPreference = self.opencv_pipeline(
                sensor = self._device_address,
                capture_width=self.hardware_resolution[0],
                capture_height=self.hardware_resolution[1],
                display_width= self._output_resolution[0],
                display_height= self._output_resolution[1],
                flip_method=2
            )

            if self._isJetson is True: 

                pipeline, apiPreference = self.nv_pipeline(
                    sensor = self._sensor_id,
                    capture_width=self.hardware_resolution[0],
                    capture_height=self.hardware_resolution[1],
                    display_width= self._output_resolution[0],
                    display_height= self._output_resolution[1],
                    flip_method=2
                )
   
            print(pipeline)
            self._capture = cv2.VideoCapture(pipeline, apiPreference)
            
            if not self._capture.isOpened():
            
                print("ERROR: could not open camera!")
                logging.error( f"an error has occured in videocapture : could not open camera" )
                return self

            self._capture.set( cv2.CAP_PROP_FRAME_WIDTH, self._output_resolution[0] )
            self._capture.set( cv2.CAP_PROP_FRAME_HEIGHT,  self._output_resolution[1] )

            self.ret, self.frame = self._capture.read()


        else: 

            self._capture = self.h264_pipeline(
                sensor = self._device_address,
                width = self._output_resolution[0],
                height = self._output_resolution[1],
                framerate = 30,
                flip_method = 0,
                address = peer_address 
            )

            self._gst_window_opened = True

            self._gst_pipeline = Gst.parse_launch( self._capture )

            self._gst_pipeline.set_state( Gst.State.PLAYING ) 


        self._thread_stopped = False

        self._thread_ = Thread( target=self._run )
        self._thread_.daemon = True
        self._thread_.start()

        return self


    def disable( self, isOpenCVRendering = True  ):

        if self._thread_ is not None:

            self._thread_stopped  = True

            try:

                self._thread_.join()

            except Exception as ex:
                
                print(ex)
                pass

            self._thread_ = None
          

        if self._capture is not None: 

            if self._IsOpenCVEnabled is True :

                self._capture.release()
                self._capture = None

            else:

                if self._gst_pipeline is not None: 

                    self._gst_pipeline.set_state(Gst.State.NULL)
                    self._gst_pipeline = None


        logging.info( f"videocapture has been disactivated" )
