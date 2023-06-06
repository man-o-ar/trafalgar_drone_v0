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
import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Camera( object ):
    
    def __init__( self, device_address = "/dev/video0", video_resolution = (320 , 240) ):

        super().__init__()

        self._device_address = device_address

        self._resolution = video_resolution

        self._pipeline = None
        self.isPlaying = False
        self._enableCV = False

        self._frame = None
        self._gst_frame = None

        self._lock = Lock()
        self._thread_stopped = True
        self._thread_ = None


    @property
    def hardware_resolution(self): 
        return ( 1280, 720 )

    @property
    def hardware_framerate(self): 
        return 30
    
    @property
    def hardware_flip( self ):
        return 0

    def _run( self ):
        
        while not self._thread_stopped:

            try:

                with self._lock:                  
                    self._read_from_cv()

            except Exception as ex:
                logging.warning( f"an error has occured in videocapture : {ex}" )
                break 
    


    def _read_from_cv(self): 

        if self._enableCV is True and self._gst_frame is not None :          
            self._frame = cv2.imencode( '.jpg', self._gst_frame )#self._bridge.cv2_to_imgmsg( self._gst_frame, encoding="bgr8" )


    def h264_pipeline( self ):

        str_pipeline = (

            f"v4l2src device={self._device_address} "
            f"! video/x-raw, width=(int){self.hardware_resolution[0]}, height=(int){self.hardware_resolution[1]} "
            f"! videoflip method={self.hardware_flip} "
            "! videoscale "    
            f"! video/x-raw, width=(int){self._resolution[0]}, height=(int){self._resolution[1]} "
            "! tee name=t "
            "! queue "
            "! videoconvert "
            "! video/x-raw, format=(string)BGR " 
            "! appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false "
            " t. "
            "! queue "
            "! videoconvert "
            #"! frei0r-filter-cartoon "
            "! x264enc speed-preset=ultrafast tune=zerolatency "
            "! h264parse config-interval=1 "
            "! rtph264pay "
            "! udpsink name=udpsink sync=false " 

        )

        return str_pipeline
    
    def set_udpsink_host(self, sink_address = "127.0.0.1", sink_port = 3000 ):
    
        if self._pipeline is not None:
            
            if self._device_address != sink_address:

                self._pipeline.set_state(Gst.State.PAUSED)

                self._udpsink.set_property("host", sink_address )
                self._udpsink.set_property("port", sink_port)

                self._pipeline.set_state(Gst.State.PLAYING)
                

    def OnNewSample(self, sink):

        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()

        caps = sample.get_caps()
        frame_width = caps.get_structure(0).get_value("width")
        frame_height = caps.get_structure(0).get_value("height")
        _, frame = buf.map(Gst.MapFlags.READ)

        # Conversion de l'image en tableau numpy
        frame = np.ndarray(
            (frame_height, frame_width, 3),
            buffer=frame.data,
            dtype=np.uint8
        )

        self._gst_frame = frame
        #Debug
        #cv2.imshow("Processed Frame", frame)
        #cv2.waitKey(1)

        return Gst.FlowReturn.OK

    def enable( self ):
        
        if self._pipeline is not None: 
            self.disable()
        
        Gst.init( None )
            
        pipe = self.h264_pipeline()

        self._pipeline = Gst.parse_launch( pipe )

        appsink = self._pipeline.get_by_name("appsink")
        appsink.connect("new-sample", self.OnNewSample)

        self._udpsink = self._pipeline.get_by_name("udpsink")

        self._pipeline.set_state( Gst.State.PLAYING ) 

        self.isPlaying = True 

        self._thread_stopped = False

        self._thread_ = Thread( target=self._run )
        self._thread_.daemon = True
        self._thread_.start()

        return self

    def pause( self, enable = True ):

        if self._pipeline is not None:
            if enable is True : 
                if self.isPlaying is False:
                    self.isPlaying = True
                    self._pipeline.set_state( Gst.State.PLAYING ) 
            else:
                if self.isPlaying is True:
                    self._enableCV = False
                    self.isPlaying = False
                    self._pipeline.set_state(Gst.State.PAUSED)



    def disable( self ):

        if self._thread_ is not None:

            self._thread_stopped  = True

            try:

                self._thread_.join()

            except Exception as ex:
                
                print(ex)
                pass

            self._thread_ = None
          
        if self._pipeline is not None: 

            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
            self._gst_frame = None
            self._frame = None


        logging.info( f"videocapture has been disactivated" )
