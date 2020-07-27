#David Buffkin
from random import random
from math import sqrt, sin, cos, pi
from panda3d.core import LVector3


day = 3600 * 24

#Constants
max_leafstem_length = .9 #Not sure the units. this is linear though
max_stem_length = 4 #cm
max_leaf_size = .6 #unitless

leaf_growth_max = .7 * 5 * day * .5  #* .5 for splitting factor?
                                    #The problem is that it should be 5 .7 health days, but
                                    #the health will be split up. Hmmmm
stem_growth_max = .8 * leaf_growth_max
droop_rate = 20 / (100 * day) #20 degrees per day per 100ml below optimal
class Leaf:

    def __init__(self, baby):

        
        self.baby = baby  #Is it a baby leaf?
        self.color = [0, .4, 0]  #All leaves have alpha = 1. this is [r, g, b]
        self.start_position = LVector3(0, 0, 0)  #This is relative to the plant stem top
        self.angleBase = 81 #This is the angle from the horizontal
        self.angleDelta = 0 #This goes from 0 to 60, and gets subtracted from the leaf angle to represent drooping
        self.angle = 81
        self.size = .05  #This is the size of the leaf. Unitless--it is the scale in panda. goes 0 -> 1
        #self.tilt = this could be harder than I thought
        self.multiplier1 = .8 + random() * .4
        self.multiplier2 = .8 + random() * .4

        self.current_leaf_growth = 0
        self.current_stem_growth = 0

    def grow_leaf(self, growth, soilwater, duration):
        self.current_leaf_growth += growth
        frac = self.current_leaf_growth  / (leaf_growth_max * self.multiplier1)
        if(frac > 1): frac = 1
        self.size = .05 + (max_leaf_size - .05) * (frac)
        self.angleBase = 90 * (.9 - frac)

        if(soilwater < optimal_soilwater[0]):
            self.angleDelta += droop_rate * (optimal_soilwater[0] - soilwater) * duration
            self.angleDelta = min(60, self.angleDelta)
        else:
            self.angleDelta -= (25 / day) * duration #25 degrees perkier per day when watered well
            self.angleDelta = max(0, self.angleDelta)
        

        self.angle = self.angleBase - self.angleDelta

    def grow_stem(self, growth, lightfrac):
        growth *= self.multiplier2
        self.current_stem_growth += growth if self.current_stem_growth / stem_growth_max < lightfrac else 0
        frac = self.current_stem_growth / stem_growth_max
        if(frac > 1): frac = 1
        self.start_position *= (max_leafstem_length * self.multiplier2 * frac / self.start_position.length())
        


    def grow(self, health, growth_amount, lightfrac, soilwater, duration):
        #if lf = 0 unlit, stem it up
        #if lf = 1 lit and small leaf, leaf it up
        #if lit and big leaf, give it up (do nothing)

        leaf_amount = .5 * growth_amount
        stem_amount = .5 * growth_amount
        '''
        if(lightfrac < 1):
            stem_amount = (.5 + .3 * (1 - lightfrac)) * growth_amount
            leaf_amount = (.5 - .3 * (1 -lightfrac)) * growth_amount'''

        self.grow_leaf(leaf_amount, soilwater, duration)
        self.grow_stem(stem_amount, lightfrac)




'''
Growth stages
germinate (stem goes up)
first two baby leaves (grow up)
next two leaves
next two
probably done now
'''

#Constants TODO these are VERY ballpark, TODO maybe tax brackets
#Also TODO this seems iffy. if one thing is off, health shouldnt be able to go up, right?
stem_rate = 2 / (.7 * 20 * day)     #cm/sec*health  takes 20 days to grow 2 cm in full light
leaf_stem_rate = stem_rate * 8      #cm/sec*health  leaf stems grow ~8x faster than main (?)

optimal_temperature = [18, 25]      #degrees celcius the temps out of which health will decline
temp_health_rate = .01 / 3600       #health/degree*second the rate at which health declines if outside of this range
                                    #or increases if inside it.

optimal_soilwater = [450 / 2, 700 / 2]      #ml " "
sw_health_rate = .1 / (100 * 3600)  #health/ml*second

optimal_airwater = [22, 45]         #ml " "
aw_health_rate = .01 / (5 * day)  #health/ml*sec

minimum_light = 250                 #Again, very ballpark. less light will be detrimental.
light_health_up_rate = .01 / 3600
light_health_down_rate = .005 / (100 * 3600)
                    
                    
                    #I think growth should be a constant based on health,
                    #Split among the leaves. then each leaf dedicates more to the stem if low light
                    #Bigger leaves will get less growth than the newer/smaller ones, since they are good already

def leaf_pair(baby, direction):
    x, y = direction
    leaf1 = Leaf(baby)
    leaf1.start_position = LVector3(x, y, random() * 3 + 2) * .001
    leaf2 = Leaf(baby)
    leaf2.start_position = LVector3(-x, -y, random() * 3 + 2) * .001
    return [leaf1, leaf2]


healthyLeafGreen= [0, .35, .02]
sicklyLeafGreen = [0, .6, 0]
healthyStemGreen= [.705, .235, .313]
sicklyStemGreen = [0, .6, 0]
green_rate = .1 / (3 * day)
deadColor = [97 / 255, 55 / 255, 10 / 255]
howLongBelow2 = 6 * day
deathTime = 5 * day

class Plant:
    #TODO when to die? maybe if go < .24 for too long or smth
    def __init__(self, node):
        
        #Plant life parameters
        self.health = .65 + random() * .1  #a value [0, 1] where 1 is perfect health, .5 is ok, 0 is terrible.
        self.cumulative_health = 0 #add to this a bit stochastically
        self.timeBelow2 = 0

        #Plant render parameters
        self.node = node
        self.stem_height = .01 #The height of the base stem
        self.leaves = [] #The set of leaves of the plant. see leaf class for more.
        self.rotation = (random() * 180, random() * 10 - 5, random() * 10 - 5)
        self.delay = random() * 2 * day #A random growth delay
        self.stemColor = [.6, .5, .2]
        self.leafColor = [0, .3, 0]
        self.colorScale = .6

        self.stemColorWhenDied = None
        self.leafColorWhenDied = None
    def change_health(self, amount):
        #change health sort of sigmoidally (towards 1, at least. to 0, probably just put to 0.)
        #ORR for now, hehe
        self.health += amount
        if self.health > 1: self.health = 1
        if self.health < .1: self.health = .1
    
    def grow(self, env_params, duration, light):

        if self.health < .2:
            self.timeBelow2 += duration

        if self.timeBelow2 > howLongBelow2:
            if self.stemColorWhenDied == None:
                self.stemColorWhenDied = self.stemColor
                self.leafColorWhenDied = self.leafColor
            deadFrac = (self.timeBelow2 - howLongBelow2) / deathTime
            if deadFrac > 1: return
            for i in [0, 1, 2]:
                self.stemColor[i] = deadFrac * deadColor[i] + (1 - deadFrac) * self.stemColorWhenDied[i]
                self.leafColor[i] = deadFrac * deadColor[i] + (1 - deadFrac) * self.leafColorWhenDied[i]
            return


        if(self.delay > 0):
            self.delay -= duration
            return

        #Update health based on env

        health_delta = 0

        if env_params['temperature'] < optimal_temperature[0]:
            health_delta -= duration * temp_health_rate * (optimal_temperature[0] - env_params['temperature'])
        elif env_params['temperature'] > optimal_temperature[1]:
            health_delta -= duration * temp_health_rate * (env_params['temperature'] - optimal_temperature[1])
        else:
            health_delta += temp_health_rate * duration

        if env_params['soilwater'] < optimal_soilwater[0]:
            health_delta -= duration * sw_health_rate * (optimal_soilwater[0] - env_params['soilwater'])
        elif env_params['soilwater'] > optimal_soilwater[1]:
            health_delta -= duration * sw_health_rate * (env_params['soilwater'] - optimal_soilwater[1])
        else:
            health_delta += sw_health_rate * duration

        if env_params['airwater'] < optimal_airwater[0]:
            health_delta -= duration * aw_health_rate * (optimal_airwater[0] - env_params['airwater'])
        elif env_params['airwater'] > optimal_airwater[1]:
            health_delta -= duration * aw_health_rate * (env_params['airwater'] - optimal_airwater[1])
        else:
            health_delta += sw_health_rate * duration

        if light > minimum_light:
            health_delta += duration * light_health_up_rate
        else:
            health_delta -= duration * light_health_down_rate * (minimum_light - light)

        self.change_health(health_delta)

        #Update plant based on health, env
        growth_amount = self.health * duration
        self.cumulative_health += growth_amount

       
        
        #if light enough, darker green.
        lightfrac = max(.001, light / minimum_light if light < minimum_light else 1)

        if lightfrac < 1:
            self.colorScale = max(0, self.colorScale - green_rate * duration)
        else:
            #print(self.colorScale, self.colorScale + green_rate * duration)
            self.colorScale = min(1, self.colorScale + green_rate * duration)
        #color scale goes up if enough light, down if not
        for i in [0, 1, 2]:
            self.stemColor[i] = (healthyStemGreen[i] * (self.colorScale)) + ((1 - self.colorScale) * sicklyStemGreen[i])
            self.leafColor[i] = healthyLeafGreen[i] * (self.colorScale) + (1 - self.colorScale) * sicklyLeafGreen[i]

        if self.cumulative_health > .7 * 1 * day and len(self.leaves) < 2:
            self.leaves += leaf_pair(True, (1, 0))
        if self.cumulative_health > .7 * 6 * day and len(self.leaves) < 4 and light > minimum_light:
            angle = (68 + random() * 5) * pi / 180
            self.leaves += leaf_pair(False, (cos(angle), sin(angle)))
        if self.cumulative_health > .7 * 17 * day and len(self.leaves) < 6 and light > minimum_light:
            angle = (124 + random() * 5) * pi / 180
            self.leaves += leaf_pair(False, (cos(angle), sin(angle)))

        growth_per = growth_amount / (1 + sqrt(len(self.leaves)))
        #Grow the stem
        if(light > minimum_light and self.stem_height < .2):
            self.stem_height += stem_rate * growth_per
        elif light < minimum_light and self.stem_height < 1.35:
            self.stem_height += stem_rate * (growth_per + len(self.leaves) * growth_per * (1 - lightfrac))
            growth_per *= lightfrac
        
            
        #Grow the leaves
        #TODO maybe diff leaves should get diff growth fraction. like newer ones get more?

        for leaf in self.leaves:
            
            leaf.grow(self.health, growth_per, lightfrac, env_params['soilwater'], duration)




