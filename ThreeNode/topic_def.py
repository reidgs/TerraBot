from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
sensor_names = ['cur', 'smoist', 'light', 'level', 'temp', 'humid']
actuator_names = ['freq', 'led', 'wpump', 'fan']

to_ard = {
    'led'   : Int32,
    'wpump' : Bool,
    'fan'   : Bool,
    'freq'  : Float32
}

from_ard = {
    'smoist' : Int32MultiArray,
    'cur'    : Float32MultiArray,
    'light'  : Int32MultiArray,
    'level'  : Float32,
    'temp'   : Int32MultiArray,
    'humid'  : Int32MultiArray,
}


to_stu = {
    'smoist' : Int32MultiArray,
    'cur'    : Float32MultiArray,
    'light'  : Int32MultiArray,
    'level'  : Float32,
    'temp'   : Int32MultiArray,
    'humid'  : Int32MultiArray,
}

from_stu = {
    'led'   : Int32,
    'wpump' : Bool,
    'fan'   : Bool,
    'freq'  : Float32
}

