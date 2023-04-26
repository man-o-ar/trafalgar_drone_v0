#!/usr/bin/env python3
########################################################################
# Filename    : __movement.py
# Description : propulsion node 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import os
import sys
import json
from time import sleep

from ..utils.__utils_objects import AVAILABLE_TOPICS, EXIT_STATE, MASTER_TOPICS
from ..components.__microcontroller import externalBoard

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8, UInt16
from geometry_msgs.msg import Vector3, Twist

class MovementNode( Node ):

        def __init__( self ):
            
            super().__init__("movement", namespace="drone_0")

            self._pub_sensors = None

            #MASTERS sub
            self._sub_master_joystick = None

            self._sub_propulsion = None
            self._sub_direction = None
            self._sub_orientation = None
            self._sub_pan_tilt = None

            self._sub_target = None
            self._sub_shutdown = None

            self._watchdog_sub = None

            self._component = None

            self._is_operator_connected = False
            self._force_commands_enabled = False

            self._start()


        def _start(self):
            
            self._declare_parameters()
            self._init_component()
            self._init_subscriptions()
            self._init_component_publisher()


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

        def _init_component(self):
            
            self._component = externalBoard( 
                self,
                self.get_parameter("verbose").value
            )
            self._component._enable()
            sleep(5)           
            self._component._reset_evolution()        


        def _init_subscriptions( self ):


            self._sub_master_joystick = self.create_subscription(
                Twist,
                MASTER_TOPICS.JOYSTICK.value,
                self._joystick_input,
                10
            )
            
            self._sub_master_joystick

            """
            self._sub_shutdown = self.create_subscription(
                String,
                f"/operator_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.SHUTDOWN.value}",#operator_{self.get_parameter('peer_index').value}_
                self._react_to_shutdown_cmd,
                10
            )

            self._sub_shutdown
            
            """
            self._sub_propulsion = self.create_subscription(
                UInt16,
                f"/operator_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.PROPULSION.value}",
                self._update_propulsion,
                10
            )
            
            self._sub_propulsion  # prevent unused variable warning


            self._sub_direction = self.create_subscription(
                Int8,
                f"/operator_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.DIRECTION.value}",
                self._update_direction,
                10
            )
            
            self._sub_direction  # prevent unused variable warning


            self._sub_orientation = self.create_subscription(
                Int8,
                f"/operator_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.ORIENTATION.value}",#operator_{self.get_parameter('peer_index').value}_
                self._update_orientation,
                10
            )

            self._sub_orientation


            self._sub_pan_tilt = self.create_subscription(
                Vector3,
                f"/operator_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.PANTILT.value}",#operator_{self.get_parameter('peer_index').value}_
                self._update_pan_tilt,
                10
            )

            self._sub_pan_tilt


            self._sub_target = self.create_subscription(
                String,
                f"/operator_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.NAVTARGET.value}",#operator_{self.get_parameter('peer_index').value}_
                self._update_navigation_target,
                10
            )

            self._sub_target 

            self._watchdog_sub = self.create_subscription(
                Bool,
                f"/drone_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.WATCHDOG.value}",
                self._react_to_connections,
                10
            )

            self._watchdog_sub   


        def _init_component_publisher( self ):

            self._sensor_publisher = self.create_publisher(
                String, 
                AVAILABLE_TOPICS.SENSOR.value,
                10
            )
            

        def _joystick_input(self, msg ):
            
            direction = msg.linear.x
            orientation = msg.angular.z

            if self._component is not None: 

                self._component._dispatch_msg( "direction", direction )
                self._component._dispatch_msg( "orientation", orientation )


        def _update_propulsion( self, msg ):

            if self._component is not None: 
                self._component._dispatch_msg( "propulsion", msg.data )


        def _update_direction( self, msg ):

            if self._component is not None: 
                self._component._dispatch_msg( "direction", msg.data )


        def _update_orientation( self, msg ):

            if self._component is not None: 
                self._component._dispatch_msg("orientation", msg.data )


        def _update_pan_tilt( self, msg ):

            if self._component is not None: 

                self._component._dispatch_msg("pan", msg.z )
                self._component._dispatch_msg("tilt", msg.x )


        def _update_navigation_target( self, msg ):
            
            if self._component is not None: 

                self._component._dispatch_msg("target", json.loads(msg.data) )


        def _send_sensors_datas( self, sensor_json ):
            
            sensor_msg = String()

            if( sensor_json["name"] is not None and sensor_json["data"] is not None ):
                
                sensor_msg.data = json.dumps( sensor_json )

                self._sensor_publisher.publish( sensor_msg )


        def _react_to_connections( self, msg ):

            self._is_operator_connected = msg.data
            #print( "operator status message received, connection ", msg.peer_connected)
            if self._is_operator_connected is False:

                if self._component is not None:
                    self._component._reset_evolution()
                    #print( "operatorConnected", self._is_operator_connected )


        def exit( self ):
            
            if self._component is not None:
                self._component._disable()
            
            print("shutdown movement")



def main(args=None):

    rclpy.init(args=args)
    movement_node_sub = None 

    try:
        
        movement_node_sub = MovementNode()

        rclpy.spin(movement_node_sub)

    except Exception as e:
        print( "an exception has been raised while spinning movement node : ", e )
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno

        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)

    except KeyboardInterrupt:
        print("user force interruption")
        
    finally:

        if movement_node_sub is not None:
            movement_node_sub.exit()
            sleep(0.1)

            movement_node_sub.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()