from enum import Enum

class AVAILABLE_TOPICS( str, Enum ):
    PROPULSION = "propulsion"
    DIRECTION = "direction"
    ORIENTATION = "orientation"
    STEER_ANGLE = "steerAngle"
    STEER_INCREMENT = "steerIncrement"
    STREAM = "videostream"
    IMU = "imu",
    PANTILT = "pantilt"
    NAVTARGET = "navtarget"
    HEARTBEAT = "heartbeat"
    WATCHDOG = "watchdog"
    SENSOR = "sensor"
    SHUTDOWN = "shutdown"
    BUZZER = "buzzer"
    JOYSTICK = "joy"

class OPERATOR(str, Enum):
    MASTER = "master"
    USER = "user"
    DRONE = "drone"
    
class EXIT_STATE(str, Enum ):
    SHUTDOWN = "shutdown"
    RESTART = "restart"