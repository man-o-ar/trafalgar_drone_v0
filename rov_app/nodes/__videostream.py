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

from ..utils.__utils_objects import AVAILABLE_TOPICS, EXIT_STATE, PEER
from ..components.__camera import Camera

class VideoStreamNode( Node ):

        def __init__( self ):

            super().__init__( "videostream", namespace=f"{PEER.DRONE}_0" )
            
            self._component = None

            self._sub_master = None
            self._sub_peer = None
            self._sub_watchdog = None
            self._pub_video = None
            self._sub_shutdown = None

            self._timer = None

            self._isMasterConnected = False

            self._is_peer_connected = False

            self._start()


        def _start( self ):

            self._declare_parameters()
            self._init_components()
            self._init_publishers()
            self._init_subscribers()

        def _react_to_shutdown_cmd(self, msg ): 

            instruction = json.loads(msg.data)
                
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
            self.declare_parameter("resolution", (320,240))  


        def _init_subscribers( self ):
            
            self._sub_master = self.create_subscription(
                String,
                f"/master/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self._react_to_master,
                qos_profile=qos_profile_sensor_data
            )
            
            self._sub_master  # prevent unused variable warning


            self._sub_peer = self.create_subscription(
                String, 
                f"/{PEER.USER}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self._on_peer_pulse,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_peer

            self._sub_watchdog = self.create_subscription(
                Bool,
                AVAILABLE_TOPICS.WATCHDOG.value,
                self._react_to_connections,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_watchdog   


        def _init_publishers( self ):

            self._pub_video = self.create_publisher(
                CompressedImage, 
                AVAILABLE_TOPICS.STREAM.value,
                qos_profile=qos_profile_sensor_data
            )
            
            self._pub_video


        def _react_to_master( self, msg ):

            master_pulse = json.loads( msg.data )
            self._is_master_connected = True if self.get_parameter('peer_index').value == master_pulse["control"] else False
        

        def _init_components(self):
               
            self._component = Camera(
                device_address = "/dev/video0",
                video_resolution = self.get_parameter("resolution").value
            )
            
            self._component.enable()

            publisher_rate = 1 / self.get_parameter("framerate").value
        
            self.timer = self.create_timer( 
                publisher_rate,
                self._stream  
            )

            self._component.set_udpsink_host("192.168.1.19", 3000)


        def _on_peer_pulse( self, pulse_msg ):
                    
            peer_pulse = json.loads( pulse_msg.data )

            if "address" in peer_pulse and peer_pulse["address"] is not None:
                self._component.set_udpsink_host( sink_address=peer_pulse["address"] )


        def _stream( self ):
            
            if( self._isMasterConnected is True ):
                    
                if( self._component is not None and self._pub_video is not None ):

                    self._component._enableCV = True

                    last_frame = self._component._frame

                    if last_frame is not None:

                        msg = CompressedImage()

                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.format = 'jpeg'

                        msg.data = last_frame.tostring()

                        self._pub_video.publish( msg ) 

            else:

                if self._component is not None:

                    self._component._enableCV = False


        def _react_to_connections( self, msg ):
            self._is_peer_connected = msg.data
            

        def exit( self ):
            
            if self._component is not None:
                self._component.disable( )




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
            
    rclpy.shutdown()


if __name__ == '__main__':
    main()