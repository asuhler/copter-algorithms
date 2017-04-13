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

    return ( atan2(z, sqrt(x * x + y * y)),atan2(y, x))

class Vars():
    placeholder = ""
    coordinate_array = []
    right = []
    left = []
    left_center = 0.0
    right_center = 0.0

vars = Vars()
try:
    pointIO = open("coordinates.txt", 'r')
    for line in pointIO:
        data = line.strip(" ").strip('\r').strip('\n').split(",")
        data = (float(math.radians(float(data[0]))),float(math.radians(float(data[1]))))
        vars.coordinate_array.append(data)
        print data


except:
    print "Failed to parse coordinate file.\r\n"
    traceback.print_exc()
    os._exit(1)


center = center_geolocation(vars.coordinate_array)
#center = (center[1], center[0])
print "Center: " + str(math.degrees(center[0])) + "," + str(math.degrees(center[1]))
for x in vars.coordinate_array:
    #print "Coordinate: " + str(math.degrees(x[0])) + "   Center: " + str(center[0])
    if (x[1])<(center[1]):
        vars.left.append(x)
    else:
        vars.right.append(x)

vars.left.append(center)
vars.right.append(center)

'''print "left: " + str(vars.left)
print "Right: " + str(vars.right)
print str(math.degrees(vars.left[0][0])) + "," + str(math.degrees(vars.left[0][1]))
print str(math.degrees(vars.left[1][0])) + "," + str(math.degrees(vars.left[1][1]))
print str(math.degrees(vars.right[0][0])) + "," + str(math.degrees(vars.right[0][1]))
print str(math.degrees(vars.right[1][0])) + "," + str(math.degrees(vars.right[1][1]))'''

vars.left_center = center_geolocation(vars.left)
vars.right_center = center_geolocation(vars.right)
print "Lcenter: " + str(math.degrees(vars.left_center[0])) + "," + str(math.degrees(vars.left_center[1]))
print "Rcenter: " + str(math.degrees(vars.right_center[0])) + ',' + str(math.degrees(vars.right_center[1]))