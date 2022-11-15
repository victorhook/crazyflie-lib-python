# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2022 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
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
from argparse import ArgumentParser, Namespace
import logging
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm


logging.basicConfig(level=logging.ERROR)


def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('-u', '--uris', help='Space separated list of uris.',
                        type=str, default=None)
    parser.add_argument('-p', '--pwm', help='Value to set propTestPWMRatio to.',
                        type=int, default=0)
    parser.add_argument('-s', '--store-persistent', help='Boolean to indicate.'
                        ' if you want to store the params persitent or not.',
                        default=False, action='store_true')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()

    if not args.uris:
        uris = ['radio://0/80/2M/E7E7E7E7E7']
        print(f'No uris specified, using default: {uris}')
    else:
        uris = args.uris.split(' ')

    param_name = 'health.propTestPWMRatio'
    propTestPWMRatio = args.pwm
    store_persistent = args.store_persistent

    print(f'Setting {param_name} to {propTestPWMRatio} for'
          f' following uris: {uris}')

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
        print('Connected to all Crazyflies, setting the parameters...')
        swarm.parallel_safe(lambda scf: scf.cf.param.set_value(param_name, str(propTestPWMRatio)))

        # Need to wait a bit here until the parameters are set.
        # TODO: Better solution than hardcore sleep?
        print('Waiting for parameters to be set...')
        time.sleep(5)

        if store_persistent:
            print('Storing the parameters persistent')
            # The callback function given to persistent_store is never called, not sure why.
            swarm.parallel_safe(lambda scf: scf.cf.param.persistent_store(param_name, callback=lambda *a: print(a)))
        else:
            print('NOT storing the parameters persistent')

    time.sleep(5)
    print('Done')