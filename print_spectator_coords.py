#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('./PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

_HOST_ = '127.0.0.1'
_PORT_ = 2000
_SLEEP_TIME_ = 1    


def main():
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()

    # print(help(t))
    # print("(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z))

    while True:
        t = world.get_spectator().get_transform()
        # coordinate_str = "(x,y) = ({},{})".format(t.location.x, t.location.y)
        coordinate_str = "(x,y,z) = ({},{},{})".format(t.location.x, t.location.y, t.location.z)
        rotation_str = "(roll,pitch,yaw) = ({},{},{})".format(t.rotation.roll, t.rotation.pitch, t.rotation.yaw)
        print(coordinate_str)
        print(rotation_str)

        coordinate_str="<waypoint x=\"{:.2f}\" y=\"{:.2f}\" z=\"{:.2f}\" pitch=\"360.0\" roll=\"0.0\" yaw=\"{:.1f}\"/>"\
            .format(t.location.x, t.location.y, t.location.z, t.rotation.yaw)
        print(coordinate_str)

        # transform for json
        transform_str = '''
        {{"transform": {{
                "pitch": "0",
                "x": {:.2f},
                "y": {:.2f},
                "yaw": {:.1f},
                "z": 1.0
            }}
        }}
        '''.format(t.location.x, t.location.y, t.rotation.yaw)
        print(transform_str)

        time.sleep(_SLEEP_TIME_)


if __name__ == '__main__':
    main()


