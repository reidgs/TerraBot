scale = {}
limits = {}
optimal = {} # Note, not all the sensors have 'optimal' levels
names = {}

scale['light_level'] = [0, 1000]
limits['light_level'] = [850, 950]
optimal['light_level'] = [860,940]
names['light_level'] = 'lights'

scale['water_level'] = [0, 200]
limits['water_level'] = [5, 200]
optimal['water_level'] = [5,200]
names['water_level'] = 'w_level'

scale['moisture'] = [0, 1000]
limits['moisture'] = [500, 650]
optimal['moisture'] = [550,600]
names['moisture'] = 'moist'

scale['humidity'] = [0, 100]
limits['humidity'] = [60, 90]
optimal['humidity'] = [70, 80]
names['humidity'] = 'humid'

scale['temperature'] = [10, 40] # Celcius
limits['temperature'] = [22, 29]
optimal['temperature'] = [25,27]
names['temperature'] = 'temp'

scale['weight'] = [0, 2000]
limits['weight'] = [300, 1500]
names['weight'] = 'weight'

scale['current'] = [0, 1000]
limits['current'] = [500, 600]
names['current'] = 'current'

scale['energy'] = [0, 15000]
limits['energy'] = [1000, 2000]
names['energy'] = 'energy'

names['led'] = 'led'
names['fan'] = 'fan'
names['pump'] = 'wpump'
