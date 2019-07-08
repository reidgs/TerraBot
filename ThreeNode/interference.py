#Interfering with sensor data
def humid_inter(x):
    return x

def temp_inter(x):
    return x

def light_inter(x):
    return int(x*3.41+13.531)  #converted to lux

def level_inter(x):
    return x

def tds_inter(x):
    return x

def cur_inter(x):
    return (x-512)*.0491 #converted to amps

#Interfering with Actuators
def led_inter(x):
    return x

def wpump_inter(x):
    return x

def npump_inter(x):
    return x

def apump_inter(x):
    return x

def fan_inter(x):
    return x

def freq_inter(x):
    return x
