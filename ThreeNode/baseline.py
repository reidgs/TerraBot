#!/usr/bin/env python


#python types
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

init_internals = {
    'volume'      : 0.0,
    'nutrient'    : 0.0,
    'light'       : 0.0,
    'temperature' : 0.0,
    'humidity'    : 0.0,
    'current'     : 0.0
}

init_actuators = {
    'led'   : 0,
    'wpump' : False,
    'npump' : False,
    'apump' : False,
    'fan'   : False,
    'freq'  : 10.0
}



