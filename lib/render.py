#!/usr/bin/env python
#David Buffkin

from direct.showbase.ShowBase import ShowBase
from panda3d.core import AmbientLight, DirectionalLight, LightAttrib, PointLight, Spotlight, PerspectiveLens
from panda3d.core import LVector3
from panda3d.core import TextNode
from direct.task import Task
from panda3d.core import loadPrcFileData 
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import ClockObject
from panda3d.core import AudioSound
from panda3d.core import GraphicsEngine, Filename
from panda3d.core import WindowProperties
from sys import exit
import random
import plant
import math
from time import sleep
from terrabot_utils import clock_time
from environment import max_daylight, day_fraction, airwater_to_humid

# Importing math constants and functions
from math import pi, sin, cos

import atexit

class Terrarium(ShowBase):

    def __init__(self, shown, t0, initTime, leafDroop, lankiness, plant_health):
        loadPrcFileData('', 'win-size 1024 768')
        loadPrcFileData("", "window-type none")
        
        ShowBase.__init__(self)

        if shown:
            self.openMainWindow(type = "onscreen")
            props = WindowProperties()
            props.setTitle('TerraBot Simulator')
            self.win.requestProperties(props)
        else:
            self.openMainWindow(type = "offscreen")

        base.disableMouse()  # Allow manual positioning of the camera
        #camera.setPosHpr(-20, 0, -3, -90, 12, 0) # Under
        camera.setPosHpr(-20, 0, 7, -90, -12, 0) # Normal
        #camera.setPosHpr(0, 0, 30, 0, -90, 0) #TOP
        
        self.pic = False
        self.loc = None
        self.shown = shown
        self.renderCount = -1

        self.start_time = t0
            
        atexit.register(self.userExit)
        self.BASE_TEXT = '''
        Pump: OFF
        Fans: OFF
        LEDs: 255
        '''

        self.BASE_TEXT2 = \
        '''
        Time : {:s}
        Light level: 0
        Temperature : 20 C
        Soil moisture : 0
        Humidity : 50%
        Volume : 3000 ml
        Weight : 100 g
        Speedup : 1x
        '''.format(clock_time(t0 + initTime))

        self.lastTime = initTime
        self.droop = leafDroop
        self.lankiness = lankiness
        self.plant_health = plant_health


        #self.accept('escape', self.userExit)
        self.accept('r', self.resetCam)

        self.loadModels()  
        self.setupLights() 
        self.setupText() 
        self.setupText2()
        self.setupSensorCam()
        self.setTankWater(0)
        self.setBackgroundColor(.8, .8, .8, 1)
        
        self.keys = {}
        for key in ['arrow_left', 'arrow_right', 'arrow_up', 'arrow_down',
                    'a', 'd', 'w', 's']:
            self.keys[key] = 0
            self.accept(key, self.push_key, [key, 1])
            self.accept('shift-%s' % key, self.push_key, [key, 1])
            self.accept('%s-up' % key, self.push_key, [key, 0])

        self.fanonSound = loader.loadSfx('sounds/fanon.wav')
        self.pumponSound = loader.loadSfx('sounds/pumpon.wav')
        self.fanonSound.setLoop(True)
        self.pumponSound.setLoop(True)

        #SetupCamera
        self.heading = -90.0
        self.pitch = -12.0
        self.camera.setPos(-20, 0, 7)
        
        self.picNextFrame = False

        self.taskMgr.add(self.update, 'main loop')


    def resetCam(self):
        camera.setPosHpr(-20, 0, 7, -90, -12, 0)
        self.heading = -90.0
        self.pitch = -12.0

    def setupSensorCam(self):
        #use the same size as the main window
        xsize, ysize = self.getSize()
        #TESTING purposes
        #self.accept("space", self.takeAndStorePic, ["test.png"])
        
        #Create the camera's buffer : GraphicsOutput
        self.camBuffer = GraphicsEngine.makeParasite(self.graphicsEngine, host=self.win, name="camera", sort=0, x_size = xsize, y_size = ysize)

        self.sensorCam = self.makeCamera(self.camBuffer, camName="sensorCam")
        
        self.sensorCam.reparentTo(render)
        self.sensorCam.setPos(0, 5.56, 4.17)
        self.sensorCam.setHpr(180, -14.74, 0)
        self.camBuffer.setClearColorActive(True)
        self.camBuffer.setClearColor((.8, .8, .8, 1))

    def takeAndStorePic(self, location):
        self.pic = True
        self.loc = location

    def setupText(self):
        self.textpanel = OnscreenText(
            text=self.BASE_TEXT, parent=base.a2dTopLeft,
            style=1, font = loader.loadFont('courier.ttf'), fg=(1, 1, 1, 1), pos=(0.06, -0.06),
            align=TextNode.ALeft, scale=.05)
    
    def setupText2(self):
        self.textpanel2 = OnscreenText(
            text=self.BASE_TEXT2, parent=base.a2dTopRight,
            style=1, font = loader.loadFont('courier.ttf'), fg=(1, 1, 1, 1), pos=(-0.06, -0.06),
            align=TextNode.ARight, scale=.05)

    def loadModels(self):
        self.terrarium = render.attachNewNode('terrarium')
        self.terrarium.reparentTo(render)
        self.terrarium.setScale(2.6)


        self.t_rings = loader.loadModel('models/Rings.egg')
        self.t_rings.reparentTo(self.terrarium)

        self.t_pump = loader.loadModel('models/Pump.egg')
        self.t_pump.reparentTo(self.terrarium)

        self.t_table = loader.loadModel('models/Table.egg')
        self.t_table.reparentTo(self.terrarium)
        self.t_table.setTransparency(True)

        self.t_glass = loader.loadModel('models/Glass.egg')
        self.t_glass.reparentTo(self.terrarium)
        self.t_glass.setTransparency(True)
        self.t_glass.setTwoSided(True)

        self.t_arduino = loader.loadModel('models/Arduino.egg')
        self.t_arduino.reparentTo(self.terrarium)

        self.t_boards = loader.loadModel('models/Boards.egg')
        self.t_boards.reparentTo(self.terrarium)

        self.t_raspi = loader.loadModel('models/Raspi.egg')
        self.t_raspi.reparentTo(self.terrarium)

        self.t_lights = loader.loadModel('models/Lights.egg')
        self.t_lights.reparentTo(self.terrarium)
        self.t_lights.setTwoSided(True)

        self.t_reservoir = loader.loadModel('models/Reservoir.egg')
        self.t_reservoir.reparentTo(self.terrarium)
        self.t_reservoir.setTransparency(True)

        self.t_reservoirLid = loader.loadModel('models/ReservoirLid.egg')
        self.t_reservoirLid.reparentTo(self.terrarium)

        self.t_reservoirWater = loader.loadModel('models/ReservoirWater.egg')
        self.t_reservoirWater.reparentTo(self.terrarium)
        self.t_reservoirWater.setTransparency(True)

        self.t_fanon = loader.loadModel('models/Fans_on.egg')
        self.t_fanon.reparentTo(self.terrarium)
        self.t_fanon.hide()

        self.t_fanonblades = loader.loadModel('models/Fans_on_blades.egg')
        self.t_fanonblades.reparentTo(self.terrarium)
        self.t_fanonblades.hide()
        self.t_fanonblades.setTransparency(True)
        
        self.t_fanoff = loader.loadModel('models/Fans_off.egg')
        self.t_fanoff.reparentTo(self.terrarium)

        self.t_growmat = loader.loadModel('models/Growmat')
        self.t_growmat.reparentTo(self.terrarium)

        self.t_piping = loader.loadModel('models/Piping')
        self.t_piping.reparentTo(self.terrarium)

        self.t_tankwater = loader.loadModel('models/Tankwater.egg')
        self.t_tankwater.reparentTo(self.terrarium)
        self.t_tankwater.setTransparency(True)

        self.t_sensors1 = loader.loadModel('models/Sensors1')
        self.t_sensors1.reparentTo(self.terrarium)

        self.t_sensors2 = loader.loadModel('models/Sensors2')
        self.t_sensors2.reparentTo(self.terrarium)

        self.t_camera = loader.loadModel('models/Camera')
        self.t_camera.reparentTo(self.terrarium)
        
        self.t_colorsBase = loader.loadModel('models/ColorsBase.egg')
        self.t_colorsBase.reparentTo(self.terrarium)
        self.t_colorsBase.setColor(.93, .93, .93, 1)
        
        self.t_colorsRed = loader.loadModel('models/ColorsRed.egg')
        self.t_colorsRed.reparentTo(self.terrarium)
        self.t_colorsRed.setColor(1, 0, 0, 1)
        
        self.t_colorsGreen = loader.loadModel('models/ColorsGreen.egg')
        self.t_colorsGreen.reparentTo(self.terrarium)
        self.t_colorsGreen.setColor(0, 1, 0, 1)
        
        self.t_colorsBlue = loader.loadModel('models/ColorsBlue.egg')
        self.t_colorsBlue.reparentTo(self.terrarium)
        self.t_colorsBlue.setColor(0, 0, 1, 1)
        
        self.t_colorsWhite = loader.loadModel('models/ColorsWhite.egg')
        self.t_colorsWhite.reparentTo(self.terrarium)
        self.t_colorsWhite.setColor(.88, .88, .88, 1)
        
        self.plants = []
        self.plantsNode = render.attachNewNode('plants')
        self.plantsNode.reparentTo(self.terrarium)
        
        for i, x in enumerate((-.56, .56)):
            for j, y in enumerate((-1.03, -.7, -.355, -.01, .34, .683, 1.03)):
                node = render.attachNewNode('lettuce' + str(i) + str(j))
                node.reparentTo(self.plantsNode)
                node.setPos(x, y, 1.14)
                node.setScale(.3)
                self.plants += [plant.Lettuce(node, self.lastTime, self.droop, self.lankiness, self.plant_health, self)]
                
        for i, x in enumerate((-.185, .185)):
            for j, y in enumerate((-1.03, -.7, -.355, -.01, .34, .683, 1.03)):
                node = render.attachNewNode('radish' + str(i) + str(j))
                node.reparentTo(self.plantsNode)
                node.setPos(x, y, 1.14)
                node.setScale(.2)
                self.plants += [plant.Radish(node, self.lastTime, self.droop, self.lankiness, self.plant_health, self)]
                
        self.reRenderPlants()


    # Panda Lighting
    def setupLights(self):
        global ambientLight

        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((.8, .8, .8, 1))
        render.setLight(render.attachNewNode(ambientLight))

        lightPositions = [(.5, 1, 1.8), (-.5, 1, 1.8), (.5, 0, 1.8), \
                          (-.5, 0, 1.8), (.5, -1, 1.8), (-.5, -1, 1.8)]
        self.lights = [PointLight('Light{}'.format(i)) for i in range(6)]
        for i, l in enumerate(self.lights):
            l.setColor((0, 0, 0, 1))
            pln = render.attachNewNode(l)
            pln.reparentTo(self.terrarium)
            pln.setPos(lightPositions[i])
            render.setLight(pln)


    def spinCameraTask(self, task):
        
        angleDegrees = task.time * 10.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 7)
        self.camera.setHpr(angleDegrees, -12, 0)
        '''
        temp = int(.5 * task.time)
        self.setFans(temp % 2 == 1) 
        '''
        return Task.cont

    
    def setWater(self, volume):
        if(volume < .5):
            self.t_reservoirWater.hide()
            return
        self.t_reservoirWater.setScale(1, 1, float(volume) / (170 * 18)) # This is (max_waterlevel * volume_rate)

    def setTankWater(self, volume):
        if(volume < .5):
            self.t_tankwater.hide()
            return
        self.t_tankwater.setScale(1, 1, volume / (170 * 18))
    

    def setFans(self, on):
        if on:
            self.t_fanoff.hide()
            self.t_fanon.show()
            self.t_fanonblades.show()
        else:
            self.t_fanoff.show()
            self.t_fanon.hide()
            self.t_fanonblades.hide()

    def setLights(self, val):
        # New grow lights are soft yellow/white
        level = 0.8*min(1, val/255.0)
        for l in self.lights:
            l.setColor((level, level, 0.8*level, 1))

    def setSoilColor(self, soilwater):
        mult = 1 - soilwater / 800.0
        if mult < 0:
            mult = 0.0
        self.t_growmat.setColorScale(.7 + mult / 3, .7 + mult / 3, .7 + mult / 3, 1)

    def setAmbient(self, time):
        global ambientLight
        sunlight = max(0.2, day_fraction(time))
        ambientLight.setColor((sunlight, sunlight, sunlight, 1))

    def fansound(self, fan):
        if not self.shown:
            self.fanonSound.stop()
            return
        if fan and self.fanonSound.status() == AudioSound.READY:
            self.fanonSound.play()
        if not fan and self.fanonSound.status() == AudioSound.PLAYING:
            self.fanonSound.stop()

    def pumpsound(self, pump):
        if not self.shown:
            self.pumponSound.stop()
            return
        if pump and self.pumponSound.status() == AudioSound.READY:
            self.pumponSound.play()
        if not pump and self.pumponSound.status() == AudioSound.PLAYING:
            self.pumponSound.stop()
            
    def reRenderPlants(self):
        # I think panda was occasionally dying b/c it was rendering too often
        self.renderCount += 1
        if (self.renderCount%5 != 0): return

        for testPlant in self.plants:
            baseStem = testPlant.stemModel
            baseStem.reparentTo(testPlant.node)
            stemFrac = testPlant.stem_height 
            baseStem.setScale(.5 + .5 * stemFrac, .5 + .5 * stemFrac, stemFrac )
            testPlant.node.setHpr(testPlant.rotation)
            sr, sg, sb = testPlant.stemColor
            lr, lg, lb = testPlant.leafColor
            baseStem.setColor(sr, sg, sb, 1) 
            #to model Leaf leafToModel on plant testPlant
            for leafToModel in testPlant.leaves:
                leaf = leafToModel.leafModel
                leaf.reparentTo(testPlant.node)
                leaf.setScale(leafToModel.size)
                rotation = None
                if leafToModel.start_position.x == 0:
                    rotation = 90 if leafToModel.start_position.y > 0 else 270
                else:
                    rotation = 180 / math.pi * math.atan(leafToModel.start_position.y / leafToModel.start_position.x)
                if(leafToModel.start_position.x < 0):
                    rotation += 180
                leaf.setHpr(rotation, 0, -leafToModel.angle)
                leaf.setPos(leafToModel.start_position + LVector3(0, 0, stemFrac))
                stem = leafToModel.stemModel
                stem.reparentTo(testPlant.node)
                stem.setPos(0, 0, stemFrac)
                leafStemLength= leafToModel.start_position.length() 
                leafStemFrac = leafStemLength / plant.max_leafstem_length
                stem.setScale(.8 * leafStemFrac, .8 * leafStemFrac, leafStemLength)
                
                #Annoying to have to get this, but here we go
                bottom = math.sqrt(leafToModel.start_position.x ** 2 + leafToModel.start_position.y ** 2)
                angle = 90 if bottom == 0 else 180 / math.pi * math.atan(leafToModel.start_position.z / bottom)
                
                stem.setHpr(rotation, 0, 90 - angle)

                #testPlant.node.setColor(r, g, b, 1)
                rm, gm, bm = leafToModel.colorRands
                #stem.setColor(.8 * sr + .2 * lr, .8 * sg + .2 * lg / 2, .8 * sb + .2 * lb / 2, 1)
                stem.setColor(.5 * sr + .5 * lr + rm, .5 * sg + .5 * lg + gm, .5 * sb + .5 * lb, 1 + bm)
                
                leaf.setColor(lr + rm, lg + gm, lb + bm, 1)


    def update_env_params(self, params, speedup, light, weight):
        self.setWater(params['volume']) #Reservoir update
        self.setLights(params['led']) #LED update
        self.setFans(params['fan']) #Fan Update
        self.setTankWater(params['tankwater']) #Tankwater update
        self.setSoilColor(params['soilwater'])
        self.setAmbient(params['time'])
        
        for plant in self.plants:
            plant.grow(params, params['time'] - self.lastTime, light)
        self.lastTime = params['time']
        
        if not self.shown: return
        
        self.reRenderPlants()
        
        #Stats panel :
        healths = [p.health for p in self.plants]
        avgH = sum(healths)/len(healths)
        avgH = int(avgH * 100)

        self.textpanel.text = \
        '''
        Pump: {}
        Fans: {}
        LEDs: {}
        
        Plant Health: {}%
        '''.format('ON' if params['wpump'] else 'off', \
            'ON' if params['fan'] else 'off', \
            params['led'], avgH)

        self.textpanel2.text = \
        '''
        Time : {:s}
        Light level: {:01.0f}
        Temperature : {:04.1f} C
        Soil moisture : {:03.1f}
        Humidity : {:02.0f}%
        Volume : {:04.1f} ml
        Weight : {:04.1f} g
        Speedup : {}x
        '''.format(clock_time(self.start_time + params['time']), light, \
                   params['temperature'], 2*params['soilwater'], \
                   params['humidity'], params['volume'], weight, speedup)

        self.fansound(params['fan'])
        self.pumpsound(params['wpump'])


    def push_key(self, key, value):
        """Stores a value associated with a key."""
        self.keys[key] = value

    def update(self, task):
        if self.shown:
            """Updates the camera based on the keyboard input."""
            delta = globalClock.getDt() * 1.4
            move_x = delta * 7 * -self.keys['a'] + delta * 3 * self.keys['d']
            move_z = delta * 7 * self.keys['s'] + delta * 3 * -self.keys['w']
            self.camera.setPos(self.camera, move_x, -move_z, 0)
            self.heading += (delta * 30 * self.keys['arrow_left'] +
                             delta * 30 * -self.keys['arrow_right'])
            self.pitch += (delta * 30 * self.keys['arrow_up'] +
                           delta * 30 * -self.keys['arrow_down'])
            self.camera.setHpr(self.heading, self.pitch, 0)
            
        if(self.picNextFrame):
            if self.loc == None:
                print("No location specified")
            else:
                self.camBuffer.saveScreenshot(Filename(self.loc))
            self.picNextFrame = False
        
        if self.pic:
            if not self.shown:
                self.reRenderPlants()
            self.pic = False
            self.picNextFrame = True
        return task.cont
    
