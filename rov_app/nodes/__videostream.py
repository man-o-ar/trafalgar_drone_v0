#!/usr/bin/env python3
########################################################################
# Filename    : __videostream.py
# Description : videostream node 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import os
import sys
from time import sleep 
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import CompressedImage # Image is the message type
from rclpy.qos import qos_profile_sensor_data

from ..utils.__utils_objects import AVAILABLE_TOPICS, EXIT_STATE
from ..components.__camera import Camera

class VideoStreamNode( Node ):

        def __init__( self ):

            super().__init__( "videostream", namespace="drone_0" )
            
            self._component = None
            self._watchdog_sub = None
            self._pub_video = None
            self._sub_shutdown = None
            self._timer = None

            self._gst_pipeline = None

            self._cv_window_opened = False
            self._gst_window_opened = False 

            self._is_operator_connected = False

            self._peer_address = None

            self._start()


        def _start( self ):

            self._declare_parameters()
            self._init_components()
            self._init_publishers()
            self._init_subscribers()

        def _react_to_shutdown_cmd(self, msg ): 

            instruction = json.loads(msg.data)
                
            operator = instruction["peer"]
            instruction = instruction["status"]

            if( instruction == EXIT_STATE.RESTART.value ):

                self._restart_instruction()

            elif ( instruction == EXIT_STATE.SHUTDOWN.value ):

                self._kill_instruction()

        def _kill_instruction():
            os.system("shutdown /s /t 1")

        def _restart_instruction():
            os.system("shutdown /r /t 1")


        def _declare_parameters( self ):

            self.declare_parameter("verbose", False)
            self.declare_parameter("peer_index", 0)
            self.declare_parameter("framerate",30)
            self.declare_parameter("resolution", (960,540))  

            self.declare_parameter( "opencv_render", True )


        def _init_subscribers( self ):
            
            self._peer_sub = self.create_subscription(
                String, 
                f"/drone_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self._on_peer_pulse,
                qos_profile=qos_profile_sensor_data
            )


            self._watchdog_sub = self.create_subscription(
                Bool,
                AVAILABLE_TOPICS.WATCHDOG.value,
                self._react_to_connections,
                qos_profile=qos_profile_sensor_data
            )

            self._watchdog_sub   


        def _init_publishers( self ):

            self._pub_video = self.create_publisher(
                CompressedImage, 
                AVAILABLE_TOPICS.STREAM.value,
                qos_profile=qos_profile_sensor_data
            )
            
            self._pub_video


        def _init_components(self):
               
            self._component = Camera(
                video_resolution = self.get_parameter("resolution").value
            )
            
            publisher_rate = 1 / self.get_parameter("framerate").value
        
            self.timer = self.create_timer( 
                publisher_rate,
                self._stream  
            )

            if( self.get_parameter("opencv_render").value is True ):
                  
                self._component.enable( peer_address = None, OpenCV_render = True )
                

        def _on_peer_pulse( self, pulse_msg ):

            if( self.get_parameter("opencv_render").value is False ):
                    
                peer_id = json.loads( pulse_msg.data )

                address = peer_id["address"]

                if address != self._peer_address:

                    self._peer_address = address
                    self._component.enable( self._peer_address, False )


        def _stream( self ):
            
            if( self.get_parameter("opencv_render").value is True ):
                    
                if( self._component is not None and self._pub_video is not None ):
                
                    last_frame = self._component._frame

                    if last_frame is not None:

                        msg = CompressedImage()

                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.format = 'jpeg'

                        msg.data = last_frame.tostring()

                        self._pub_video.publish( msg ) 


        def _react_to_connections( self, msg ):

            operator_status = msg.data

            if operator_status is False:

                if self._is_operator_connected is True:
                    
                    if self._component is not None and self._component._thread_stopped is False :
                        self._component.disable()

            else:

                if self._is_operator_connected is False:
                    
                    if self._component is not None and self._component._thread_stopped is True :
                        
                        if( self.get_parameter("opencv_render").value is True ):
                  
                            self._component.enable( self._peer_address, True )


            self._is_operator_connected = operator_status
            

        def exit( self ):
            
            if self._component is not None:
                self._component.disable( self.get_parameter("opencv_render").value )




def main(args=None):
        
    rclpy.init(args=args)

    videostream_node = None

    try:

        videostream_node = VideoStreamNode()

        rclpy.spin(videostream_node)

    except Exception as e:
        print( "an exception has been raised while spinning videostream node : ", e )
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno

        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)

    except KeyboardInterrupt:
        print("user force interruption")
        
    finally:
        
        if videostream_node is not None:

            videostream_node.exit()
            sleep(0.1)

            videostream_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()