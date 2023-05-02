#!/usr/bin/env python3
########################################################################
# Filename    : __movement.py
# Description : propulsion node 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import os
import sys
import numpy as np
import json
from time import sleep

from ..utils.__utils_objects import AVAILABLE_TOPICS, EXIT_STATE, MASTER_TOPICS
from ..components.__microcontroller import externalBoard

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, UInt16
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy

from rclpy.qos import qos_profile_sensor_data

class MovementNode( Node ):

        def __init__( self ):
            
            super().__init__("movement", namespace="drone_0")

            self._pub_sensors = None
            
            self._sub_master = None             

            self._sub_propulsion = None
            self._sub_direction = None
            self._sub_orientation = None
            self._sub_pan_tilt = None
            self._sub_buzzer = None

            self._watchdog_sub = None

            self._component = None

            #joystick reader
            self._sub_joystick = None
            self._propulsion_level = 50
            self._b_joy_pressed = False
            self._ccw_joy_pressed = False
            self._cw_joy_pressed = False
            self._propulsion_joy_pressed = False

            self._is_operator_connected = False
            self._is_master_connected = False

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

            self._sub_master = self.create_subscription(
                String,
                f"/master/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self._react_to_master,
                qos_profile=qos_profile_sensor_data
            )
            
            self._sub_master  # prevent unused variable warning


            self._sub_joystick = self.create_subscription( 
                Joy, 
                f"/master/{AVAILABLE_TOPICS.JOYSTICK.value}", 
                self._joystick_read, 
                qos_profile=qos_profile_sensor_data 
            )

            self._sub_joystick

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
                qos=qos_profile_sensor_data
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
                qos_profile_sensor_data
            )

            self._sub_orientation


            self._sub_pan_tilt = self.create_subscription(
                Vector3,
                f"/operator_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.PANTILT.value}",#operator_{self.get_parameter('peer_index').value}_
                self._update_pan_tilt,
                qos_profile_sensor_data
            )

            self._sub_pan_tilt

            self._sub_buzzer = self.create_subscription(
                UInt16,
                f"/operator_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.BUZZER.value}",#operator_{self.get_parameter('peer_index').value}_
                self._enable_buzzer,
                qos_profile_sensor_data
            )

            self._sub_buzzer 


        def _init_component_publisher( self ):

            self._sensor_publisher = self.create_publisher(
                String, 
                AVAILABLE_TOPICS.SENSOR.value,
                qos_profile = qos_profile_sensor_data
            )
            

        def _react_to_master( self, msg ):

            master_pulse = json.loads( msg.data )
            self._is_master_connected = True if self.get_parameter('peer_index').value == master_pulse["control"] else False
            

        def _update_propulsion( self, msg ):
            
            if self._is_master_connected is False and self._component is not None: 
                self._component._dispatch_msg( "propulsion", msg.data )


        def _update_direction( self, msg ):

            if self._is_master_connected is False and self._component is not None: 
                self._component._dispatch_msg( "direction", msg.data )


        def _update_orientation( self, msg ):

            if self._is_master_connected is False and self._component is not None: 
                self._component._dispatch_msg("orientation", msg.data )


        def _update_pan_tilt( self, msg ):

            if self._is_master_connected is False and self._component is not None: 

                self._component._dispatch_msg("pan", msg.z )
                self._component._dispatch_msg("tilt", msg.x )


        def _enable_buzzer( self, msg = 500 ):

            if self._component is not None: 
                self._component._dispatch_msg("buzzer", msg.data )


        def _update_navigation_target( self, msg ):
            
            if self._component is not None: 
                self._component._dispatch_msg("target", json.loads(msg.data) )


        def _send_sensors_datas( self, sensor_json ):
            
            sensor_msg = String()

            if( sensor_json["name"] is not None and sensor_json["data"] is not None ):
                
                sensor_msg.data = json.dumps( sensor_json )

                self._sensor_publisher.publish( sensor_msg )


        def _joystick_read( self, msg ):

            self.b_button( msg.buttons[1] )
            self.bottom_triggers(msg.buttons[6], msg.buttons[7]  )
            self.arrow_pad( msg.axes[ 5 ], msg.axes[ 4 ] )
            self.left_joystick( msg.axes[ 1 ] ) 
            self.right_joystick( msg.axes[ 2 ], msg.axes[ 3 ] )


        def b_button( self, ButtonState ):

            if ButtonState == 1:  
                
                if self._b_joy_pressed != True:
                    self._b_joy_pressed = True

                    self._enable_buzzer()
  
            else: 
                self._b_joy_pressed = False


        def bottom_triggers( self, L2, R2 ): 

            if L2 == 1:  
                
                if self._ccw_joy_pressed != True:
                    self._ccw_joy_pressed = True
                
                    if self._component is not None: 
                        self._component._dispatch_msg("orientation", -10 )

            else: 
                self._ccw_joy_pressed = False

            if R2 == 1:  
                
                if self._cw_joy_pressed != True:
                    self._cw_joy_pressed = True

                    if self._component is not None: 
                        self._component._dispatch_msg("orientation", 10 )
                 
            else: 
                self._cw_joy_pressed = False


        def arrow_pad( self, vertical_arrow, horizontal_arrow ):

            propulsion = vertical_arrow

            if propulsion != 0: 
                
                if self._propulsion_joy_pressed != True:
                    
                    self._propulsion_joy_pressed = True

                    if propulsion > 0: 
                        self._propulsion_level += 25
                    else:
                        self._propulsion_level -= 25

                    self._propulsion_level = np.clip( self._propulsion_level, 50, 200 )

                    if self._component is not None: 
                        self._component._dispatch_msg( "propulsion", self._propulsion_level )

            else: 

                self._propulsion_joy_pressed = False



        def left_joystick( self, direction ):

            if direction > 0 : 
                direction = 1

            elif direction < 0:
                direction = -1
            else: 
                direction = 0

            if self._component is not None: 
                self._component._dispatch_msg( "direction", direction )


        def right_joystick(self, x_value, y_value ):
            
            angle_min = 0
            angle_max = 180

            pan_angle = 90

            if x_value != 0:
                pan_angle = angle_min + ( angle_max - angle_min ) * ( x_value + 1 ) / 2

            # Limitez l'angle aux valeurs minimum et maximum
            pan_angle = np.clip( pan_angle, angle_min, angle_max )


            tilt_angle = 90
            # Calculez l'angle en fonction des valeurs de l'axe X et Y pour le deuxième servomoteur
            if y_value != 0:
                tilt_angle = angle_min + ( angle_max - angle_min ) * ( y_value + 1 ) / 2

            # Limitez l'angle aux valeurs minimum et maximum
            tilt_angle = np.clip( tilt_angle, angle_min, angle_max )

            #self.get_logger().info(f"send camera angle pan: {pan_angle} / tilt: {tilt_angle}")

            if self._component is not None: 

                self._component._dispatch_msg("pan", pan_angle )
                self._component._dispatch_msg("tilt", tilt_angle )


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