#!/usr/bin/env python
#David Buffkin

from direct.showbase.ShowBase import ShowBase
from panda3d.core import AmbientLight, DirectionalLight, LightAttrib, PointLight, Spotlight, PerspectiveLens
from panda3d.core import LVector3
from panda3d.core import TextNode
from direct.task import Task
from panda3d.core import loadPrcFileData 
from direct.gui.OnscreenText import OnscreenText
from sys import exit


# Importing math constants and functions
from math import pi, sin, cos

class Terrarium(ShowBase):

    def __init__(self):
        loadPrcFileData('', 'win-size 1024 768')
        ShowBase.__init__(self)


        base.disableMouse()  # Allow manual positioning of the camera
        #camera.setPosHpr(-20, 0, -3, -90, 12, 0) # Under
        #.setAspectRatio for clearer images?
        camera.setPosHpr(-20, 0, 7, -90, -12, 0) # Normal
        #camera.setPosHpr(0, 0, 30, 0, -90, 0) #TOP

        self.BASE_TEXT = '''
        Pump: OFF
        Fans: OFF
        LEDs: 255
        '''

        self.accept('escape', self.userExit)

        self.loadModels()  # Load and position our models
        self.setupLights()  # Add some basic lighting
        self.setupText() 


        #self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
        #self.setWater(1200)
        #self.setLights(200)


    def setupText(self):
        self.textpanel = OnscreenText(
            text=self.BASE_TEXT, parent=base.a2dTopLeft,
            style=1, fg=(1, 1, 1, 1), pos=(0.06, -0.06),
            align=TextNode.ALeft, scale=.05)

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



    def update_env_params(self, params):
        self.setWater(params['volume']) #Reservoir update
        self.setLights(params['led']) #LED update
        self.setFans(params['fan']) #Fan Update
        #Tankwater?
        self.t_growmat.setColorScale(1, 1, 1, 1) #Darken Soil TODO actaully do this haha
        #Stats panel :
        self.textpanel.text = \
        '''
        Pump: {}
        Fans: {}
        LEDs: {}
        '''.format('ON' if params['wpump'] else 'off', \
            'ON' if params['fan'] else 'off', \
            params['led'])
        #Pump particles?
        
