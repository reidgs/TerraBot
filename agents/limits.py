scale = {}
limits = {}
names = {}

scale['light_level'] = [0, 255]
limits['light_level'] = [200, 240]
names['light_level'] = 'lights'

scale['water_level'] = [0, 200]
limits['water_level'] = [5, 20]
names['water_level'] = 'w_level'

scale['moisture'] = [0, 1000]
limits['moisture'] = [300, 450]
names['moisture'] = 'moist'

scale['humidity'] = [0, 100]
limits['humidity'] = [50, 70]
names['humidity'] = 'humid'

scale['temperature'] = [10, 40] # Celcius
limits['temperature'] = [18, 27]
names['temperature'] = 'temp'

scale['current'] = [0, 1000]
limits['current'] = [500, 600]
names['current'] = 'current'

scale['energy'] = [0, 15000]
limits['energy'] = [1000, 2000]
names['energy'] = 'energy'

names['lighting_fsm'] = 'lighting fsm'
names['watering_fsm'] = 'watering fsm'
names['humidity_fsm'] = 'humidity fsm'
names['temperature_fsm'] = 'temperature fsm'
