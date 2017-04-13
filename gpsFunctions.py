import math
from math import cos, sin, atan2, sqrt
from geopy.distance import vincenty, VincentyDistance
import geopy


def bearingToPoint(destLat, destLon, sourceLat, sourceLon):  # calculates bearing based on current position and next position

    destLat = math.radians(destLat)
    destLon = math.radians(destLon)
    currLat = math.radians(sourceLat)
    currLon = math.radians(sourceLon)
    dLon = destLon - currLon
    y = math.sin(dLon) * math.cos(destLat)
    x = math.cos(currLat ) *math.sin(destLat) -math.sin(currLat) * math.cos(destLat) * math.cos(dLon)
    brng = math.atan2(y, x) * (180 / math.pi)
    brng = (brng) % 360

    return brng


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

    return [ atan2(z, sqrt(x * x + y * y)),atan2(y, x)]


def newCoordinates(distance, bearing, currLat, currLon):  # calculates the new coordinates based on current position, bearing, and distance to move

    current = geopy.Point(currLat, currLon)
    destination = VincentyDistance(meters=distance).destination(current, bearing)
    return [destination.latitude,destination.longitude]


def distanceToCoordinates(destLat, destLon, currLat, currLon):                                              #Gives the distance from one GPS point to another using GEOPY
    current = geopy.Point(currLat, currLon)
    dest = geopy.Point(destLat, destLon)
    relation = vincenty(current, dest).meters
    return relation