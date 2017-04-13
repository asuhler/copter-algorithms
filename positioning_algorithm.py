import math
import traceback
import os
from gpsFunctions import center_geolocation, bearingToPoint, distanceToCoordinates, newCoordinates




class Vars():
    placeholder = ""
    coordinate_array = []
    right = []
    left = []
    left_center = 0.0
    right_center = 0.0

    copter_max_distance = float(8) #meters

vars = Vars()
try:
    pointIO = open("coordinates.txt", 'r')
    for line in pointIO:
        data = line.strip(" ").strip('\r').strip('\n').split(",")
        data = [float(math.radians(float(data[0]))),float(math.radians(float(data[1])))]
        vars.coordinate_array.append(data)


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


def printEverything():
    for x in vars.coordinate_array:
        print str(math.degrees(x[0])) + "," + str(math.degrees(x[1]))
    print str(math.degrees(vars.left_center[0])) + "," + str(math.degrees(vars.left_center[1]))
    print str(math.degrees(vars.right_center[0])) + ',' + str(math.degrees(vars.right_center[1]))


#Check distance between copters for maximum distances

dist = distanceToCoordinates(math.degrees(vars.left_center[0]), math.degrees(vars.left_center[1]), math.degrees(vars.right_center[0]), math.degrees(vars.right_center[1]))
print "dist: " + str(dist)
if dist > vars.copter_max_distance:
    #get bearing from right center to left center
    bearing = bearingToPoint(math.degrees(vars.left_center[0]), math.degrees(vars.left_center[1]), math.degrees(vars.right_center[0]), math.degrees(vars.right_center[1]))
    #print bearing
    move_dist = float((dist-vars.copter_max_distance)/2)   #grab half distance required move copter
    new_coords = newCoordinates(move_dist, bearing, math.degrees(vars.right_center[0]), math.degrees(vars.right_center[1]))
    vars.right_center[0] = math.radians(new_coords[0])
    vars.right_center[1] = math.radians(new_coords[1])
    #print new_coords

    #move left center
    bearing = bearingToPoint(math.degrees(vars.right_center[0]), math.degrees(vars.right_center[1]), math.degrees(vars.left_center[0]), math.degrees(vars.left_center[1]))
    new_coords = newCoordinates(move_dist, bearing, math.degrees(vars.left_center[0]), math.degrees(vars.left_center[1]))
    vars.left_center[0] = math.radians(new_coords[0])
    vars.left_center[1] = math.radians(new_coords[1])



    print "Lcenter: " + str(math.degrees(vars.left_center[0])) + "," + str(math.degrees(vars.left_center[1]))
    print "Rcenter: " + str(math.degrees(vars.right_center[0])) + ',' + str(math.degrees(vars.right_center[1]))

#printEverything()