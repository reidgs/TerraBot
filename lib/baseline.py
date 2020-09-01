from terrabot_utils import dtime_to_seconds

#TODO add error catching support, used_energy?
class Baseline:
    def __init__(self, filename):
        self.params = {'start' : 0, 'temperature' : 20, 'humidity' : 50, \
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
                
                if key in ['wpump', 'fan']:
                    self.params[key] = (val == 'on')
                elif key in ['leaf_droop', 'lankiness', 'plant_health']:
                    self.params[key] = min(1, max(0, float(val)))
                elif key == 'start':
                    self.params['start'] = (int(val) if (val.find('-') < 0) else
                                            dtime_to_seconds(val))
                elif key not in self.params.keys():
                    print("invalid parameter name: {}".format(key))
                    return
                elif key == 'led':
                    self.params[key] = int(float(val))
                else:
                    self.params[key] = float(val)
    
    def __str__(self):
        return str(self.params)
