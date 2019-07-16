sensor_names = ['tds', 'cur', 'light', 'level', 'temp', 'hum']
actuator_names = ['freq', 'led', 'wpump', 'npump', 'apump', 'fan', 'cam']

pub_types = {
    'tds'   : Int32,
    'cur'   : Float32,
    'light' : Int32,
    'level' : Int32,
    'temp'  : Int32,
    'hum'   : Int32,
    'freq'  : Float32,
    'led'   : Int32,
    'wpump' : Bool,
    'npump' : Bool,
    'apump' : Bool,
    'fan'   : Bool,
    'cam'   : String
}

sub_types = {
    'tds'   : Int32,
    'cur'   : Int32,
    'light' : Int32,
    'level' : Int32,
    'temp'  : Int32,
    'hum'   : Int32,
    'freq'  : Float32,
    'led'   : Int32,
    'wpump' : Bool,
    'npump' : Bool,
    'apump' : Bool,
    'fan'   : Bool,
    'cam'   : Bool
}
