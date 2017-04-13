from math import cos, sin, atan2, sqrt
import math
import traceback
import os



def center_geolocation(geolocations):
    """
    Provide a relatively accurate center lat, lon returned as a list pair, given
    a list of list pairs.
    ex: in: geolocations = ((lat1,lon1), (lat2,lon2),)
        out: (center_lat, center_lon)
    """
    x = 0
    y = 0
    z = 0

    for lat, lon in geolocations:
        lat = float(lat)
        lon = float(lon)
        x += cos(lat) * cos(lon)
        y += cos(lat) * sin(lon)
        z += sin(lat)

    x = float(x / len(geolocations))
    y = float(y / len(geolocations))
    z = float(z / len(geolocations))

    return (atan2(y, x), atan2(z, sqrt(x * x + y * y)))

class Vars():
    placeholder = ""
    coordinate_array = []

vars = Vars()
try:
    pointIO = open("coordinates.txt", 'r')
    for line in pointIO:
        data = line.strip(" ").strip('\r').strip('\n').split(",")
        data = (float(math.radians(float(data[0]))),float(math.radians(float(data[1]))))
        vars.coordinate_array.append(data)
        print data
    center = center_geolocation(vars.coordinate_array)
    center = (math.degrees(center[1]), math.degrees(center[0]))
    print "Center: " + str(center)


except:
    print "Failed to parse coordinate file.\r\n"
    traceback.print_exc()
    os._exit(1)