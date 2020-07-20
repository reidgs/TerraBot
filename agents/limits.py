scale = {}
limits = {}
names = {}

scale['light_level'] = [0, 1000]
limits['light_level'] = [800, 950]
names['light_level'] = 'lights'

scale['water_level'] = [0, 200]
limits['water_level'] = [5, 200]
names['water_level'] = 'w_level'

scale['moisture'] = [0, 1000]
limits['moisture'] = [450, 650]
names['moisture'] = 'moist'

scale['humidity'] = [0, 100]
limits['humidity'] = [75, 90]
names['humidity'] = 'humid'

scale['temperature'] = [10, 40] # Celcius
limits['temperature'] = [24, 30]
names['temperature'] = 'temp'

scale['current'] = [0, 1000]
limits['current'] = [500, 600]
names['current'] = 'current'

scale['energy'] = [0, 15000]
limits['energy'] = [1000, 2000]
names['energy'] = 'energy'

names['led'] = 'led'
names['fan'] = 'fan'
names['pump'] = 'wpump'
