#!/usr/bin/env python

import os


for filename in os.listdir('ArduinoCode'):
    print(os.path.join('ArduinoCode', filename))

