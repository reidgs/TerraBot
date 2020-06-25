#TODO add error catching support, used_energy?
class Baseline:
    def __init__(self, filename):
        self.params = {'time' : 0, 'temperature' : 20, 'humidity' : 50, \
                        'smoist' : 350, 'wlevel' : 160, 'tankwater' : 0, \
                        'pump' : False, 'fan' : False, 'led' : 0}
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
                
                if key not in self.params.keys():
                    print("invalid parameter name: {}".format(key))
                    return
                if key == 'pump':
                    self.pump = (val == 'on')
                elif key == 'fan':
                    self.fan = (val == 'on')
                else:
                    self.params[key] = int(val)
    
    def __str__(self):
        return str(self.params)
