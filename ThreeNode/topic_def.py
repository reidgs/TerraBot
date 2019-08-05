from std_msgs.msg import Int32,Bool,Float32,String
sensor_names = ['tds', 'cur', 'light', 'level', 'temp', 'humid']
actuator_names = ['freq', 'led', 'wpump', 'npump', 'apump', 'fan']

to_ard = {
    'led'   : Int32,
    'wpump' : Bool,
    'npump' : Bool,
    'apump' : Bool,
    'fan'   : Bool,
    'freq'  : Float32
}

from_ard = {
    'tds'   : Int32,
    'cur'   : Float32,
    'light' : Int32,
    'level' : Float32,
    'temp'  : Int32,
    'humid'   : Int32,
}


to_stu = {
    'tds'   : Int32,
    'cur'   : Float32,
    'light' : Int32,
    'level' : Float32,
    'temp'  : Int32,
    'humid'   : Int32,
}

from_stu = {
    'led'   : Int32,
    'wpump' : Bool,
    'npump' : Bool,
    'apump' : Bool,
    'fan'   : Bool,
    'freq'  : Float32
}

