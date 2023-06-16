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
    
    def __init__( self, device_address = "/dev/video0", video_resolution = (360 , 240) ):

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

        self._isJetson = False

        self._isUDPSinkActive = False
        self._isAppSinkActive = False

        self._isHighQualityCodec = False


    @property
    def hardware_resolution(self): 
        return ( 720, 480 )

    @property
    def hardware_framerate(self): 
        return 30
    
    @property
    def hardware_flip( self ):
        return 0

    @property
    def config_interval( self ):
        return 30
    

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
            self._frame = cv2.imencode( '.jpg', self._gst_frame )[1].tobytes()#self._bridge.cv2_to_imgmsg( self._gst_frame, encoding="bgr8" )

            #Debug
            #cv2.imshow("Processed Frame", frame)
            #cv2.waitKey(1)


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
            #"! frei0r-filter-cartoon "
            "! x264enc speed-preset=ultrafast tune=zerolatency "
            "! video/x-h264, stream-format=byte-stream "
            f"! rtph264pay config-interval={self.config_interval}"
            "! udpsink name=udpsink sync=false async=false" 
            " t. "
            "! queue "
            "! videoconvert "
            "! identity drop-allocation=true "
            "! appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false"

        )

        return str_pipeline
    

    def h265_pipeline( self ):

        str_pipeline = (

            f"v4l2src device={self._device_address} "
            f"! video/x-raw, width=(int){self.hardware_resolution[0]}, height=(int){self.hardware_resolution[1]} "
            f"! videoflip method={self.hardware_flip} "
            "! videoscale "    
            f"! video/x-raw, width=(int){self._resolution[0]}, height=(int){self._resolution[1]} "
            "! tee name=t "
            "! queue "
            "! videoconvert "
            #"! frei0r-filter-cartoon "
            f"! x265enc speed-preset=ultrafast tune=zerolatency "
            "! video/x-h265, stream-format=byte-stream "
            f"! rtph265pay config-interval={self.config_interval} "
            "! udpsink name=udpsink sync=false async=false" 
            " t. "
            "! queue "
            "! videoconvert "
            "! identity drop-allocation=true "
            "! appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false"

        )

        return str_pipeline
    

    def jetson_simple_camera( self ):


        pipeline = (

                "nvarguscamerasrc "
                "! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720 "
                "! nvvidconv flip-method=2 "   
                "! video/x-raw, width=(int)320, height=(int)180 "
                "! tee name=t "
                "! queue "
                "! x264enc speed-preset=ultrafast tune=zerolatency "
                "! video/x-h264, stream-format=byte-stream "
                #"! video/x-raw(memory:NVMM) "
                #"! nvv4l2h265enc name=enc1 control-rate=variable_bitrate bitrate=20000000 profile=Main"
                f"! rtph264pay config-interval={self.config_interval} "
                "! udpsink name=udpsink sync=false async=false " 
                "t. "
                "! queue "
                "! videoconvert "
                "! video/x-raw, format=(string)BGRx "
                "! videoconvert "
                "! video/x-raw, format=(string)BGR "
                "! identity drop-allocation=true "
                "! appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false"

            )

        if self._isHighQualityCodec is True:
            pipeline = (

                "nvarguscamerasrc "
                "! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720 "
                "! nvvidconv flip-method=2 "   
                "! video/x-raw, width=(int)320, height=(int)180 "
                "! tee name=t "
                "! queue "
                "! x265enc speed-preset=ultrafast tune=zerolatency "
                "! video/x-h265, stream-format=byte-stream "
                #"! video/x-raw(memory:NVMM) "
                #"! nvv4l2h265enc name=enc1 control-rate=variable_bitrate bitrate=20000000 profile=Main"
                f"! rtph265pay config-interval={self.config_interval} "
                "! udpsink name=udpsink sync=false async=false " 
                "t. "
                "! queue "
                "! videoconvert "
                "! video/x-raw, format=(string)BGRx "
                "! videoconvert "
                "! video/x-raw, format=(string)BGR "
                "! identity drop-allocation=true "
                "! appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false"

            )

        return pipeline
    
    
    def OnHostConnect(self, sink_address = "127.0.0.1", sink_port = 3000 ):
    
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
        _, buffer = buf.map(Gst.MapFlags.READ)

        frame = np.frombuffer(buffer.data, dtype=np.uint8)
        frame = frame.reshape((frame_height, frame_width, 3))

        self._gst_frame = frame

        buf.unmap(buffer)

        frame = None
        sample = None
        buf = None
            
        return Gst.FlowReturn.OK


    def enable( self ):
        
        if self._pipeline is not None: 
            self.disable()
        
        Gst.init( None )
        
        pipe = self.h265_pipeline() if self._isHighQualityCodec is True else self.h264_pipeline()
        
        if self._isJetson is True:
            pipe = self.jetson_simple_camera()

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
