#!/usr/bin/env python

#types
farduino_types = {
    'led'   : int,
    'wpump' : bool,
    'npump' : bool,
    'apump' : bool,
    'fan'   : bool,
    'freq'  : float,
    'tds'   : int,
    'cur'   : int,
    'light' : int,
    'level' : float,
    'temp'  : int,
    'humid' : int,
}


#initial values
init_sensors = {
    'tds'   : 0
    'cur'   : 0,
    'level' : 0.0,
    'light' : 0,
    'temp'  : 0
    'humid' : 0
}

init_actuators = {
    'led'   : 0
    'wpump' : False
    'npump' : False
    'apump' : False
    'fan'   : False
    'freq'  : 1.0
}



