from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray

sensor_names = ['weight', 'smoist', 'light', 'level', 'temp', 'humid']
actuator_names = ['led', 'wpump', 'fan', 'freq', 'camera']

actuator_types = {
    'led'   : Int32,
    'wpump' : Bool,
    'fan'   : Bool,
    'freq'  : String,
    'camera': String }

sensor_types = {
    'weight' : Float32MultiArray,
    'smoist' : Int32MultiArray,
    'light'  : Int32MultiArray,
    'level'  : Float32,
    'temp'   : Int32MultiArray,
    'humid'  : Int32MultiArray,
}

