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
from sys import exit


# Importing math constants and functions
from math import pi, sin, cos

import atexit

class Terrarium(ShowBase):

    def __init__(self):
        loadPrcFileData('', 'win-size 1024 768')
        ShowBase.__init__(self)


        base.disableMouse()  # Allow manual positioning of the camera
        #camera.setPosHpr(-20, 0, -3, -90, 12, 0) # Under
        #.setAspectRatio for clearer images?
        camera.setPosHpr(-20, 0, 7, -90, -12, 0) # Normal
        #camera.setPosHpr(0, 0, 30, 0, -90, 0) #TOP
        
        self.pic = False
        self.loc = None
            
        atexit.register(self.userExit)
        self.BASE_TEXT = '''
        Pump: OFF
        Fans: OFF
        LEDs: 255
        '''
        self.BASE_TEXT2 = '''
        Time : 0 seconds
        Temperature : 20 degrees
        Soil water : 0 ml 
        Humidity : 50 ml
        Volume : 3000 ml
        Current Speedup : 1x
        '''


        #self.accept('escape', self.userExit)
        self.accept('r', self.resetCam)

        self.loadModels()  # Load and position our models
        self.setupLights()  # Add some basic lighting
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

        self.fanonSound = loader.loadSfx('sounds/fanon.mp3')
        self.pumponSound = loader.loadSfx('sounds/pumpon.mp3')
        self.fanonSound.setLoop(True)
        self.pumponSound.setLoop(True)

        #SetupCamera
        self.heading = -90.0
        self.pitch = -12.0
        self.camera.setPos(-20, 0, 7)

        self.taskMgr.add(self.update, 'main loop')
        #self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
        #self.setWater(1200)
        #self.setLights(200)

    def resetCam(self):
        camera.setPosHpr(-20, 0, 7, -90, -12, 0)
        self.heading = -90.0
        self.pitch = -12.0

    def setupSensorCam(self):
        #for now, just use the same size as the main window
        xsize, ysize = self.getSize()
        #TESTING purposes
        #self.accept("space", self.takeAndStorePic, ["test.png"])
        #Create the camera's buffer : GraphicsOutput
        self.camBuffer = GraphicsEngine.makeParasite(self.graphicsEngine, host=self.win, name="camera", sort=0, x_size = xsize, y_size = ysize)
        #Create the camera
        self.sensorCam = self.makeCamera(self.camBuffer, camName="sensorCam")
        #Put the camera in position (Screw around with this till it's right)
        self.sensorCam.reparentTo(render)
        self.sensorCam.setPos(0, 5.56, 4.17)
        self.sensorCam.setHpr(180, -14.74, 0)

    def takeAndStorePic(self, location):
        #self.t_table.hide()
        #print(Filename(location))
        self.pic = True
        self.loc = location
        #self.camBuffer.saveScreenshot(Filename(location))
        #self.t_table.show()

    def setupText(self):
        self.textpanel = OnscreenText(
            text=self.BASE_TEXT, parent=base.a2dTopLeft,
            style=1, fg=(1, 1, 1, 1), pos=(0.06, -0.06),
            align=TextNode.ALeft, scale=.05)
    
    def setupText2(self):
        self.textpanel2 = OnscreenText(
            text=self.BASE_TEXT2, parent=base.a2dTopRight,
            style=1, fg=(1, 1, 1, 1), pos=(-0.06, -0.06),
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
        #self.t_table.setTwoSided(True)

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

        
        '''self.marker = loader.loadModel('models/marker.egg')
        self.marker.reparentTo(self.terrarium)
        self.marker.setScale(.05)
        self.marker.setPos(.5, 1, 1.9)'''
        

    # Panda Lighting
    def setupLights(self):
        
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((.8, .8, .8, 1))
        render.setLight(render.attachNewNode(ambientLight))

        # Explicitly set the environment to not be lit
        #self.env.setLightOff()

        lightPositions = [(.5, 1, 1.8), (-.5, 1, 1.8), (.5, 0, 1.8), \
                          (-.5, 0, 1.8), (.5, -1, 1.8), (-.5, -1, 1.8)]
        self.lights = [PointLight('Light{}'.format(i)) for i in range(6)]
        for i, l in enumerate(self.lights):
            #Initial light values TODO Attenuation?
            l.setColor((0, 0, 0, 1)) # Start off
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
        self.t_reservoirWater.setScale(1, 1, volume / (170 * 18)) # This is (max_waterlevel * volume_rate)

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
        max_level = 255
        for l in self.lights:
            l.setColor((1 * (val / 255.0), 0, .5 * (val / 255.0), 1))

    def setSoilColor(self, soilwater):
        mult = 1 - soilwater / 800.0
        if mult < 0:
            mult = 0.0
        self.t_growmat.setColorScale(.7 + mult / 3, .7 + mult / 3, .7 + mult / 3, 1)

    def fansound(self, fan):
        if fan and self.fanonSound.status() == AudioSound.READY:
            self.fanonSound.play()
        if not fan and self.fanonSound.status() == AudioSound.PLAYING:
            self.fanonSound.stop()

    def pumpsound(self, pump):
        if pump and self.pumponSound.status() == AudioSound.READY:
            self.pumponSound.play()
        if not pump and self.pumponSound.status() == AudioSound.PLAYING:
            self.pumponSound.stop()


    def update_env_params(self, params, speedup):
        self.setWater(params['volume']) #Reservoir update
        self.setLights(params['led']) #LED update
        self.setFans(params['fan']) #Fan Update
        self.setTankWater(params['tankwater']) #Tankwater update
        self.setSoilColor(params['soilwater'])
        #Stats panel :

        self.textpanel.text = \
        '''
        Pump: {}
        Fans: {}
        LEDs: {}
        '''.format('ON' if params['wpump'] else 'off', \
            'ON' if params['fan'] else 'off', \
            params['led'])

        self.textpanel2.text = \
        '''
        Time : {:04.2f} seconds
        Temperature : {:04.2f} degrees
        Soil water : {:04.2f} ml 
        Humidity : {:04.2f} ml
        Volume : {:04.2f} ml
        Current Speedup : {}x
        '''.format(params['time'], params['temperature'], params['soilwater'] * 2, \
            params['airwater'], params['volume'], speedup)

        self.fansound(params['fan'])
        self.pumpsound(params['wpump'])


    def push_key(self, key, value):
        """Stores a value associated with a key."""
        self.keys[key] = value

    def update(self, task):
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
        
        if(self.pic):
            if self.loc == None:
                print("No location specified")
            else:
                self.camBuffer.saveScreenshot(Filename(self.loc))
            self.pic = False
        
        #print(self.camera.getPos())
        #print(self.camera.getHpr())
        return task.cont
    
