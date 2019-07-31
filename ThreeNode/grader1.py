#!/usr/bin/env python

init_internals = {
    'volume'      : 0.0,
    'nutrient'    : 0.0,
    'light'       : 0.0,
    'temperature' : 0.0,
    'humidity'    : 0.0,
    'current'     : 0.0
}

init_actuators = { 
    'wpump' : True, 
    'apump' : True,
    'led'   : 100,
    'fan'   : True
    'npump' : False
}


expected_actuators = {
    'wpump' : True,
    'apump' : True,
    'led'   : 100,
    'fan'   : True
    'npump' : False
}
