#!/usr/bin/env python

# Copyright (c) 2023 Okanagan Visualization & Interaction (OVI) Lab
# The University of British Columbia, BC, Canada
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Script used to translate vehicle location txt file into an xml file."""

import re
from lxml.etree import Element, SubElement, tostring, ElementTree

# Change the parameters appropriately for your use case
input_file = 'routegenerator/raw_waypoints/study_route_2.txt'
output_file = 'routegenerator/route_data/study_route_2.xml'
route_id = 1
town = "Town04"

with open(input_file, 'r') as file:
    data = file.read().splitlines()

routes = Element('routes')
route = SubElement(routes, 'route')
route.attrib['id'] = str(route_id)
route.attrib['town'] = town

pattern = r'Transform\(Location\(x=(-?\d+\.\d+), y=(-?\d+\.\d+), z=(-?\d+\.\d+)\), Rotation\(pitch=(-?\d+\.\d+), yaw=(-?\d+\.\d+), roll=(-?\d+\.\d+)\)\)'

for line in data:
    match = re.search(pattern, line)

    if match:
        waypoint = SubElement(route, 'waypoint')
        waypoint.attrib['x'] = match.group(1)
        waypoint.attrib['y'] = match.group(2)
        waypoint.attrib['z'] = match.group(3)
        waypoint.attrib['pitch'] = match.group(4)
        waypoint.attrib['yaw'] = match.group(5)
        waypoint.attrib['roll'] = match.group(6)

xmlstr = tostring(routes, pretty_print=True, xml_declaration=True, encoding='UTF-8')

with open(output_file, "wb") as f:
    f.write(xmlstr)