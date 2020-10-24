import os
import subprocess as sp
from behavior import *
from transitions import Machine

class StreamAV(Behavior):
    stream = None
    devnull = None

    def __init__(self):
        super(StreamAV, self).__init__("StreamAVBehavior")
        self.fsm = Machine(self, states=["Halt", "Streaming"],
                           initial="Halt", ignore_invalid_triggers=True)

        self.fsm.add_transition("enable", "Halt", "Streaming")
        self.fsm.add_transition("disable", "Streaming", "Halt")
        self.fsm.on_enter_Streaming("startStreaming")
        self.fsm.on_enter_Halt("turnOffActuator")

    def startStreaming(self):
        terrabotdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        print(terrabotdir)
        if (not self.devnull): self.devnull = open(os.devnull, "wb")
        #self.stream =sp.Popen(terrabotdir+"/stream/stream-av",
        #                      stdout=self.devnull, stderr=sp.STDOUT)
        self.stream = sp.Popen("ls")

    def stopStreaming(self):
        if (self.stream):
            self.stream.terminate()
            self.stream.wait()
            self.stream = None

    def turnOffActuator(self): self.stopStreaming()

    def start(self):
        print("Enable: %s" %self.name)
        self.trigger("enable")

    def stop(self):
        print("Disable: %s" %self.name)
        self.trigger("disable")
