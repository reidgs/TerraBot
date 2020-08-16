#TODO add error catching support, used_energy?
class Baseline:
    def __init__(self, filename):
        self.params = {'time' : 0, 'temperature' : 20, 'humidity' : 50, \
                        'smoist' : 400, 'wlevel' : 140, 'tankwater' : 0, \
                        'wpump' : False, 'fan' : False, 'led' : 0, \
                        'leaf_droop' : 0, 'lankiness' : 0, 'plant_health' : 1}
        if not filename: return
        with open(filename) as reader:
            
            for line in reader:
                
                #Remove comments
                line = line.split('#')[0].strip(' \n')
                if line == '': continue
                if line.find('=') == -1: 
                    print("invalid basline syntax")
                    exit()
                #Get key/value
                pair = line.split('=')
                key = pair[0].strip(' ')
                val = pair[1].strip(' ')
                
                if key == 'wpump':
                    self.params['wpump'] = (val == 'on')
                elif key == 'fan':
                    self.params['fan'] = (val == 'on')
                elif key == 'leaf_droop':
                    self.params['leaf_droop'] = min(1, max(0, float(val)))
                elif key == 'lankiness':
                    self.params['lankiness'] = min(1, max(0, float(val)))
                elif key == 'plant_health':
                    self.params['plant_health'] = min(1, max(0, float(val)))
                elif key == 'time':
                    if val.find('-') >= 0:
                        day = int(val[:val.find('-')])
                        hour, minute, second = val[val.find('-') +1:].split(':')
                        hour = int(hour)
                        minute = int(minute)
                        second = int(second)
                        self.params['time'] = (3600 * 24 * day) + (3600 * hour) + (60 * minute) + second
                    else:
                        self.params['time'] = int(val)
                elif key not in self.params.keys():
                    print("invalid parameter name: {}".format(key))
                    return
                else:
                    self.params[key] = int(val)
    
    def __str__(self):
        return str(self.params)
