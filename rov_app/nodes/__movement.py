#!/usr/bin/env python3
########################################################################
# Filename    : __movement.py
# Description : propulsion node 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import os
import sys
import math
import numpy as np
import json
import socket  
from time import sleep

from ..utils.__utils_objects import AVAILABLE_TOPICS, EXIT_STATE, SENSORS_TOPICS, PEER
from ..components.__microcontroller import externalBoard

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8, UInt16
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy

from rclpy.qos import qos_profile_sensor_data

class MovementNode( Node ):

        def __init__( self ):
            
            super().__init__("movement", namespace=f"{PEER.DRONE}_0")

            self._address = ""

            self._sensors_id = None

            self._pub_sensors = None
            
            self._sub_master = None             

            self._isGamePlayEnable = False 

            self._sub_watchdog = None

            self._sub_gameplay = None
            self._sub_propulsion = None
            self._sub_direction = None
            self._sub_orientation = None
            self._sub_pan_tilt = None
            self._sub_buzzer = None

            self._component = None

            #joystick reader
            self._sub_joystick = None
            self._propulsion_level = 50
            self._b_joy_pressed = False
            self._l2_joy_pressed = False
            self._r2_joy_pressed = False
            self._verticalArrow_joy_pressed = False
            self._horizontalArrow_joy_pressed = False
            
            self._is_peer_connected = False
            self._is_master_connected = False

            self._last_direction = 0
            self._last_steering_angle = 90
            self._last_thrust = 50

            self._last_tilt_angle = 90
            self._last_pan_angle = 90

            self._start()


        @property
        def min_thrust_level(self):
            return 25
        
        @property
        def max_thrust_level(self):
            return 100

        def _start(self):

            self.get_local_ip()
            self._declare_parameters()
            self._init_component()
            self._init_subscribers()
            self._init_publishers()

        def get_local_ip( self ):

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            try:

                s.connect(('8.8.8.8', 80))
                self._address = s.getsockname()[0]
            except socket.error:
                # Si la connexion échoue, nous renvoyons l'adresse IP de la machine locale
                self._address = socket.gethostbyname(socket.gethostname())
            finally:
                s.close()

        def OnShutdownCommand(self, msg ): 

            instruction = json.loads(msg.data)
                
            peer = instruction["peer"]
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

        def _init_component(self):
            
            self._component = externalBoard( self )
            self._component._enable()
            sleep(0.5)           
            self._component._reset_evolution()        


        def _init_subscribers( self ):

            self._sub_master = self.create_subscription(
                String,
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self.OnMasterPulse,
                qos_profile=qos_profile_sensor_data
            )
            
            self._sub_master  # prevent unused variable warning


            self._sub_joystick = self.create_subscription( 
                Joy, 
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.JOYSTICK.value}", 
                self._joystick_read, 
                qos_profile=qos_profile_sensor_data 
            )

            self._sub_joystick

            
            self._sub_shutdown = self.create_subscription(
                String,
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.SHUTDOWN.value}",
                self.OnShutdownCommand,
                10
            )

            self._sub_shutdown
            

            self._sub_gameplay  = self.create_subscription(
                String,
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.GAMEPLAY.value}",
                self._unlock_peer_control,
                qos_profile=qos_profile_sensor_data
            )
            
            self._sub_gameplay  # prevent unused variable warning


            self._sub_propulsion = self.create_subscription(
                UInt16,
                f"/{PEER.USER.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.PROPULSION.value}",
                self._update_propulsion,
                qos_profile=qos_profile_sensor_data
            )
            
            self._sub_propulsion  # prevent unused variable warning


            self._sub_direction = self.create_subscription(
                Int8,
                f"/{PEER.USER.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.DIRECTION.value}",
                self._update_direction,
                qos_profile=qos_profile_sensor_data
            )
            
            self._sub_direction  # prevent unused variable warning

            self._sub_orientation = self.create_subscription(
                Int8,
                f"/{PEER.USER.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.ORIENTATION.value}",
                self._update_orientation,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_orientation


            self._sub_pan_tilt = self.create_subscription(
                Vector3,
                f"/{PEER.USER.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.PANTILT.value}",
                self._update_pan_tilt,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_pan_tilt

            self._sub_buzzer = self.create_subscription(
                UInt16,
                f"/{PEER.USER.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.BUZZER.value}",
                self._enable_buzzer,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_buzzer 

            self._sub_watchdog = self.create_subscription(
                Bool,
                f"/{PEER.USER.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.WATCHDOG.value}",
                self.OnLiveOperators,
                10
            )

            self._sub_watchdog


        def _init_publishers( self ):

            self._pub_sensors = self.create_publisher(
                String, 
                AVAILABLE_TOPICS.SENSOR.value,
                qos_profile = qos_profile_sensor_data
            )
            
            self._pub_sensors

        def OnMasterPulse( self, msg ):

            master_pulse = json.loads( msg.data )
            self._is_master_connected = True if self.get_parameter('peer_index').value == master_pulse["control"] else False
        
        
        def _unlock_peer_control( self, msg ):

            gameplay_unlock = json.loads( msg.data )
            
            if( "index" in gameplay_unlock and "enable" in gameplay_unlock):
                drone_index = gameplay_unlock["index"]
                current_index = self.get_parameter('peer_index').value 

                if( drone_index == current_index):
                    self._isGamePlayEnable = gameplay_unlock["enable"]


        def _update_propulsion( self, msg ):
            
            if self._is_master_connected is False and self._component is not None: 

                if(  self._isGamePlayEnable is True ):

                    self._propulsion_level += msg.data
                    self._propulsion_level = np.clip(self._propulsion_level, self.min_thrust_level, self.max_thrust_level)

                    self._component._dispatch_msg( "propulsion", int(self._propulsion_level)  )


        def _update_direction( self, msg ):

            if self._is_master_connected is False and self._component is not None: 
                if(  self._isGamePlayEnable is True ):
                    
                    update_direction = msg.data
                    
                    if update_direction != self._last_direction :
                        self._last_direction = update_direction
                        self._component._dispatch_msg( "direction", int(update_direction) )
                        

        def _update_orientation( self, msg ):

            if self._is_master_connected is False and self._component is not None: 
                
                if(  self._isGamePlayEnable is True ):
                    self._component._dispatch_msg("steerIncrement", int(msg.data) )


        def _update_pan_tilt( self, msg ):

            if self._is_master_connected is False and self._component is not None: 

                if( self._isGamePlayEnable is True ):

                    update_pan = msg.z
                    update_tilt = msg.x

                    if update_pan != self._last_pan_angle: 
                        self._last_pan_angle = update_pan
                        self._component._dispatch_msg("pan", int(update_pan) )
                    
                    if update_tilt != self._last_tilt_angle:
                        self._last_tilt_angle = update_tilt
                        self._component._dispatch_msg("tilt", int(update_tilt) )


        def _enable_buzzer( self, frequency = 500 ):

            if self._component is not None: 
                self._component._dispatch_msg("buzzer", frequency )


        def _update_navigation_target( self, msg ):
            
            if self._component is not None: 
                self._component._dispatch_msg("target", json.loads(msg.data) )


        def _send_sensors_datas( self, sensor_json ):

            sensor_msg = String()
            
            if self._sensors_id is None:
                self._sensors_id = self.get_parameter("peer_index").value

            sensor_json[f"{SENSORS_TOPICS.IP}"] = f"{self._address}"
            #self.get_logger().info(sensor_json[f"{SENSORS_TOPICS.IP}"] )
            sensors_datas = {
                "index" : self._sensors_id,
                "datas" : sensor_json
            }

            sensor_msg.data = json.dumps( sensors_datas )
            self._pub_sensors.publish( sensor_msg )


        def _joystick_read( self, msg ):

            if( self._is_master_connected is True ):
                
                self.b_button( msg.buttons[1] )
                self.bottom_triggers(msg.buttons[6], msg.buttons[7]  )
                self.arrow_pad( msg.axes[ 4 ], msg.axes[ 5 ] )
                self.left_joystick( msg.axes[ 0 ], msg.axes[ 1 ] ) 
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
                
                if self._l2_joy_pressed != True:
                    self._l2_joy_pressed = True

                    self._propulsion_level -= 25

                    self._propulsion_level = np.clip(self._propulsion_level, self.min_thrust_level, self.max_thrust_level)

                    if self._component is not None: 
                        self._component._dispatch_msg("propulsion", int(self._propulsion_level) )

            else: 

                self._l2_joy_pressed = False

            if R2 == 1:  
                
                if self._r2_joy_pressed != True:
                    self._r2_joy_pressed = True

                    self._propulsion_level += 25
                    self._propulsion_level = np.clip(self._propulsion_level, self.min_thrust_level, self.max_thrust_level)

                    if self._component is not None: 
                        self._component._dispatch_msg("propulsion", int(self._propulsion_level) )
                 
            else: 
                self._r2_joy_pressed = False


        def arrow_pad( self, horizontal_arrow, vertical_arrow ):
            
            angle_min = 0
            angle_max = 180
        
            if vertical_arrow != 0: 
                
                if self._verticalArrow_joy_pressed != True:
                    self._verticalArrow_joy_pressed = True

                    update_tilt = self._last_tilt_angle + np.sign(vertical_arrow) * 10
                    update_tilt = np.clip( update_tilt, angle_min, angle_max )

                    if( update_tilt != self._last_tilt_angle ):
                        self._last_tilt_angle = update_tilt
                        self._component._dispatch_msg("tilt", int(update_tilt) )

            else: 

                self._verticalArrow_joy_pressed = False

            if horizontal_arrow != 0: 
                
                if self._horizontalArrow_joy_pressed != True:
                    self._horizontalArrow_joy_pressed = True

                    update_pan = self._last_pan_angle + np.sign(horizontal_arrow) * 10
                    update_pan = np.clip( update_pan, angle_min, angle_max )

                    if( update_pan != self._last_pan_angle ):
                        self._last_pan_angle = update_pan
                        self._component._dispatch_msg("pan", int( update_pan ) )

                else: 

                    self._horizontalArrow_joy_pressed = False


        def left_joystick( self, x_value, y_value ):
            
            direction =  y_value
            
            if direction != 0:
                direction = np.sign( direction )

            if self._component is not None: 
                
                if direction != self._last_direction:
                    self._last_direction = direction
                    self._component._dispatch_msg( "direction", int(direction) )


        def right_joystick(self, x_value, y_value ):
            
            angle_min = 0
            angle_max = 180
            orientation_angle = 90

            if x_value != 0:
                orientation_angle = math.floor(angle_min + ( angle_max - angle_min ) * ( x_value + 1 ) / 2)

            if orientation_angle != self._last_steering_angle:
                self._last_steering_angle = orientation_angle
                self._component._dispatch_msg("steerAngle", int(orientation_angle) )


        def right_joystick_camera(self, x_value, y_value ):
            
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

                if pan_angle != self._last_pan_angle:
                    self._last_pan_angle = pan_angle
                    self._component._dispatch_msg("pan", int(pan_angle) )
                
                if tilt_angle != self._last_tilt_angle:
                    self._last_tilt_angle = tilt_angle
                    self._component._dispatch_msg("tilt", int(tilt_angle) )


        def OnLiveOperators( self, msg ):

            self._is_peer_connected = msg.data

            if self._is_peer_connected is False:

                if self._component is not None:
                    
                    if self._is_master_connected is False:

                        self._component._reset_evolution()


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