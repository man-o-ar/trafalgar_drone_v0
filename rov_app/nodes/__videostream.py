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

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage # Image is the message type
from rclpy.qos import qos_profile_sensor_data

from ..utils.__utils_objects import AVAILABLE_TOPICS, EXIT_STATE, PEER
from ..components.__camera import Camera

class VideoStreamNode( Node ):

        def __init__( self ):

            super().__init__( "videostream", namespace=f"{PEER.DRONE.value}_0" )
            
            self._component = None

            self._sub_peer = None
            self._sub_watchdog = None
            self._pub_video = None
            self._sub_shutdown = None

            self._timer = None

            self._is_master_connected = False
            self._is_peer_connected = False
            self._peer_address = "127.0.0.1"

            self._start()

        @property
        def CV_Framerate(self):
            return 30
        
        def _start( self ):

            self._declare_parameters()
            self._init_components()
            self._init_publishers()
            self._init_subscribers()

        def OnShutdownCommand(self, msg ): 

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

            self.declare_parameter("peer_index", 0)
            self.declare_parameter("resolution", (320,240))  


        def _init_subscribers( self ):
            
            self._sub_watchdog = self.create_subscription(
                String,
                AVAILABLE_TOPICS.WATCHDOG.value,
                self.OnPeersConnections,
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



        def _init_components(self):

            self._is_master_connected = True

            self._component = Camera(
                device_address = "/dev/video0",
                video_resolution = self.get_parameter("resolution").value
            )
            
            self._component.enable()
            
            publisher_rate = 1 / self.CV_Framerate
        
            self.timer = self.create_timer( 
                publisher_rate,
                self._stream  
            )



        def _stream( self ):

            if( self._is_master_connected is True ):
                
                if self._component.isPlaying is False: 
                    self._component.pause(False)

                if( self._component is not None and self._pub_video is not None ):
                    

                    self._component._enableCV = True

                    last_frame = self._component._frame

                    if last_frame is not None:

                        msg = CompressedImage()

                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.format = 'jpeg'

                        msg.data = last_frame#.tostring()

                        self._pub_video.publish( msg ) 

     

            else:

                if self._component is not None:

                    self._component._enableCV = False


        def OnPeersConnections( self, msg ):

            peers = json.loads(msg.data)

            if PEER.MASTER.value in peers:
                self._is_master_connected = peers[PEER.MASTER.value]["isConnected"]

            if PEER.USER.value in peers:
                self._is_peer_connected = peers[PEER.USER.value]["isConnected"]
                peer = peers[PEER.USER.value]["address"]

                if peer is not None:
                    self.OnPeerConnect(peer)

            if self._is_master_connected is False and self._is_peer_connected is False:
                self.OnNoPeers()

            
        def OnPeerConnect( self, peer_address = None ):

            if peer_address is not None and peer_address != self._peer_address:
                self._peer_address = peer_address
                self._component.OnHostConnect( sink_address=self._peer_address )
 
        def OnNoPeers( self ):

            if self._component is not None:
                
                if self._component.isPlaying is True:
                    self._component.pause(True)


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
            
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()