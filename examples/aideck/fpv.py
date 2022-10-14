#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018-2022 Bitcraze AB
#
#  Crazyflie Python Library
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Example script which can be used to fly the Crazyflie in "FPV" mode
using the Flow deck and the AI deck.

The example shows how to connect to a Crazyflie over the WiFi and
use both CPX and CRTP for communication over WiFI.

When the application is started the Crazyflie will hover at 0.3 m. The
Crazyflie can then be controlled by using keyboard input:
 * Move by using the arrow keys (left/right/forward/backwards)
 * Adjust the right with w/s (0.1 m for each keypress)
 * Yaw slowly using a/d (CCW/CW)
 * Yaw fast using z/x (CCW/CW)

The demo is ended by closing the application.

For the example to run the following hardware is needed:
 * Crazyflie 2.1
 * Crazyradio PA
 * Flow v2 deck
 * AI deck 1.1

Before you start the script please ensure:
    1. You have flashed stm32, nrf and esp32 with correct firmware.
    2. GAP8 has wifi-streaming firmware.
    3. You're on the same wifi as the aideck.

You can then invoke script by:
    CFLIB_URI=tcp://192.168.4.1:5000 python examples/aideck/fpv.py

"""

from collections import namedtuple
from cfclient.utils.input import JoystickReader
import logging
import struct
import sys
import threading

import numpy as np

import cflib.crtp
from cflib.cpx import CPXFunction
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

try:
    from sip import setapi
    setapi('QVariant', 2)
    setapi('QString', 2)
except ImportError:
    pass

from PyQt5 import QtCore, QtWidgets, QtGui

from go import Joystick, Target

logging.basicConfig(level=logging.INFO)

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')


CAM_HEIGHT = 244
CAM_WIDTH = 324
# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.3

if len(sys.argv) > 1:
    URI = sys.argv[1]


class ImageDownloader(threading.Thread):
    def __init__(self, cpx, cb):
        threading.Thread.__init__(self)
        self.daemon = True
        self._cpx = cpx
        self._cb = cb

    def run(self):
        while True:
            p = self._cpx.receivePacket(CPXFunction.APP)
            [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', p.data[0:11])
            if (magic == 0xBC):
                imgStream = bytearray()
                while len(imgStream) < size:
                    p = self._cpx.receivePacket(CPXFunction.APP)
                    imgStream.extend(p.data)

                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
                self._cb(bayer_img)


Target = namedtuple('Target', ['roll', 'pitch', 'yaw', 'thrust'])


class Joystick:
    '''
    This class reads connected joystick and calls callback function
    "on_new_data" when new data has arrived.
    '''

    def __init__(self, on_new_data: callable = None) -> None:
        '''
        Parameters:
            on_new_data - Callback function that takes argument:
                Tuple[roll: float, pitch: float, yaw: float, thrust: int]
        '''
        self._jr = JoystickReader(False)
        self._reading: Target = Target(0, 0, 0, 0)
        self._on_new_data = on_new_data

    def start(self, input_map: str = 'PS3_Mode_1') -> bool:
        '''
        Tries to connect to the first device found.
        Returns True on success, False if no device is found.

        Parameters:
            input_map - A string with the correct mapping to use for joystick.
        '''
        devices = self._jr.available_devices()
        if not devices:
            print('Found no devices!')
            return False

        print(f'Found {len(devices)} devices:')
        for dev in devices:
            print(f'  - {dev.name}')

        # Just choose first device found
        dev = devices[0]
        print(f'Choosing deivce {dev.name}')

        if self._on_new_data is None:
            print('No callback supplied for new data!')
            self._on_new_data = self._print_new_data

        # Add callbacks for JoystickReader
        self._jr.device_error.add_callback(lambda err: print(err))
        self._jr.input_updated.add_callback(self._new_data_cb)

        self._jr.set_input_map(dev.name, input_map)
        self._jr.start_input(dev.name)

        print(f'Started joystick with device {dev.name}')

        return True

    def read(self) -> Target:
        return self._reading

    def _print_new_data(self, target: Target) -> None:
        print(target)

    def _new_data_cb(self, roll: float, pitch: float, yaw: float, thrust: int):
        # Value ranges with PS3_Mode_1. These are probably different for each mapping.
        #        Low  Mid  Max
        # Yaw:    0  -200 -450
        # Thrust: 0       52000
        # Pitch:  -30   0   30
        # Roll:   -30  0  30
        self._reading = Target(roll, pitch, yaw, thrust)
        self._on_new_data(self._reading)


class MainWindow(QtWidgets.QWidget):

    def __init__(self, URI):
        QtWidgets.QWidget.__init__(self)

        self.setWindowTitle('Crazyflie / AI deck FPV demo')

        self.mainLayout = QtWidgets.QVBoxLayout()

        self.image_frame = QtWidgets.QLabel()
        self.mainLayout.addWidget(self.image_frame)

        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.addWidget(QtWidgets.QLabel('Position (X/Y/Z)'), 0, 0, 1, 1, QtCore.Qt.AlignLeft)
        self.gridLayout.addWidget(QtWidgets.QLabel('Pose (roll/pitch/yaw)'), 1, 0, 1, 1, QtCore.Qt.AlignLeft)

        self.labels = {
            'stateEstimate.x': {
                'widget': QtWidgets.QLabel('X'),
                'x_grid': 0, 'y_grid': 1,
                'alignment': QtCore.Qt.AlignLeft
            },
            'stateEstimate.y': {
                'widget': QtWidgets.QLabel('Y'),
                'x_grid': 0, 'y_grid': 2,
                'alignment': QtCore.Qt.AlignLeft
            },
            'stateEstimate.z': {
                'widget': QtWidgets.QLabel('Z'),
                'x_grid': 0, 'y_grid': 3,
                'alignment': QtCore.Qt.AlignLeft
            },
            'stabilizer.roll': {
                'widget': QtWidgets.QLabel('roll'),
                'x_grid': 1, 'y_grid': 1,
                'alignment': QtCore.Qt.AlignLeft
            },
            'stabilizer.pitch': {
                'widget': QtWidgets.QLabel('pitch'),
                'x_grid': 1, 'y_grid': 2,
                'alignment': QtCore.Qt.AlignLeft
            },
            'stabilizer.yaw': {
                'widget': QtWidgets.QLabel('yaw'),
                'x_grid': 1, 'y_grid': 3,
                'alignment': QtCore.Qt.AlignLeft
            }
        }

        for name in self.labels:
            w = self.labels[name]
            self.gridLayout.addWidget(w['widget'], w['x_grid'], w['y_grid'], 1, 1, w['alignment'])

        self.mainLayout.addLayout(self.gridLayout)

        self.setLayout(self.mainLayout)

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Connect callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)

        self.joystick = Joystick(self._new_joystick_data)

        self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}

        # Connect to the Crazyflie
        self.cf.open_link(URI)

        if not self.joystick.start():
            print('Failed to find connected joystick!')

        if not self.cf.link:
            print('Could not connect to Crazyflie')
            sys.exit(1)

        if not hasattr(self.cf.link, 'cpx'):
            print('Not connecting with WiFi')
            self.cf.close_link()
        else:
            self._imgDownload = ImageDownloader(self.cf.link.cpx, self.updateImage)
            self._imgDownload.start()

            self.hoverTimer = QtCore.QTimer()
            self.hoverTimer.timeout.connect(self.sendHoverCommand)
            self.hoverTimer.setInterval(100)
            self.hoverTimer.start()


    def updateImage(self, image):
        i = QtGui.QImage(image, CAM_WIDTH, CAM_HEIGHT, QtGui.QImage.Format_Grayscale8).scaled(324*2, 244*2)
        self.image_frame.setPixmap(QtGui.QPixmap.fromImage(i))

    def keyPressEvent(self, event):
        if (not event.isAutoRepeat()):
            if (event.key() == QtCore.Qt.Key_Left):
                self.updateHover('y', 1)
            if (event.key() == QtCore.Qt.Key_Right):
                self.updateHover('y', -1)
            if (event.key() == QtCore.Qt.Key_Up):
                self.updateHover('x', 1)
            if (event.key() == QtCore.Qt.Key_Down):
                self.updateHover('x', -1)
            if (event.key() == QtCore.Qt.Key_A):
                self.updateHover('yaw', -70)
            if (event.key() == QtCore.Qt.Key_D):
                self.updateHover('yaw', 70)
            if (event.key() == QtCore.Qt.Key_Z):
                self.updateHover('yaw', -200)
            if (event.key() == QtCore.Qt.Key_X):
                self.updateHover('yaw', 200)
            if (event.key() == QtCore.Qt.Key_W):
                self.updateHover('height', 0.1)
            if (event.key() == QtCore.Qt.Key_S):
                self.updateHover('height', -0.1)

    def keyReleaseEvent(self, event):
        if (not event.isAutoRepeat()):
            if (event.key() == QtCore.Qt.Key_Left):
                self.updateHover('y', 0)
            if (event.key() == QtCore.Qt.Key_Right):
                self.updateHover('y', 0)
            if (event.key() == QtCore.Qt.Key_Up):
                self.updateHover('x', 0)
            if (event.key() == QtCore.Qt.Key_Down):
                self.updateHover('x', 0)
            if (event.key() == QtCore.Qt.Key_A):
                self.updateHover('yaw', 0)
            if (event.key() == QtCore.Qt.Key_D):
                self.updateHover('yaw', 0)
            if (event.key() == QtCore.Qt.Key_W):
                self.updateHover('height', 0)
            if (event.key() == QtCore.Qt.Key_S):
                self.updateHover('height', 0)
            if (event.key() == QtCore.Qt.Key_Z):
                self.updateHover('yaw', 0)
            if (event.key() == QtCore.Qt.Key_X):
                self.updateHover('yaw', 0)

    def sendHoverCommand(self):
        self.cf.commander.send_hover_setpoint(
            self.hover['x'], self.hover['y'], self.hover['yaw'],
            self.hover['height'])

    def updateHover(self, k, v):
        if (k != 'height'):
            self.hover[k] = v * SPEED_FACTOR
        else:
            self.hover[k] += v

    def disconnected(self, URI):
        print('Disconnected')
        sys.exit(1)

    def connected(self, URI):
        print('We are now connected to {}'.format(URI))

        # The definition of the logconfig can be made before connecting
        lp = LogConfig(name='Position', period_in_ms=100)
        lp.add_variable('stateEstimate.x')
        lp.add_variable('stateEstimate.y')
        lp.add_variable('stateEstimate.z')
        lp.add_variable('stabilizer.roll')
        lp.add_variable('stabilizer.pitch')
        lp.add_variable('stabilizer.yaw')

        try:
            self.cf.log.add_config(lp)
            lp.data_received_cb.add_callback(self.pos_data)
            lp.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

    def pos_data(self, timestamp, data, logconf):
        for name in data:
            self.labels[name]['widget'].setText('{:.02f}'.format(data[name]))

    def closeEvent(self, event):
        if (self.cf is not None):
            self.cf.close_link()

    def _new_joystick_data(self, target: Target) -> None:
        # Value ranges with PS3_Mode_1. These are probably different for each mapping.
        #        Low  Mid  Max
        # Yaw:    0  -200 -450
        # Thrust: 0       52000
        # Pitch:  -30   0   30
        # Roll:   -30  0  30
        if target.yaw < -150:
            self.updateHover('yaw', -70)
        elif target.yaw > 150:
            self.updateHover('yaw', -70)

        if target.pitch < 0:
            self.updateHover('x', -1)
        elif target.pitch > 0:
            self.updateHover('x', 1)

        if target.roll < 0:
            self.updateHover('y', 1)
        elif target.roll > 0:
            self.updateHover('y', -1)

        # TODO: Thrust goes from 0-52000, so can't go down.
        if target.thrust > 0:
            pass

if __name__ == '__main__':
    appQt = QtWidgets.QApplication(sys.argv)
    win = MainWindow(URI)
    win.show()
    appQt.exec_()
