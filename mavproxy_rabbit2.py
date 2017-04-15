#!/usr/bin/env python
'''battery commands'''

import time, math
import threading
import pika
import sys
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from geopy.distance import vincenty, VincentyDistance
import geopy
import math
import traceback

class vars(object):
    username = 'UAS'
    password = 'UAS'
    host = '127.0.0.1'
    vhost = 'UASHost'
    exchange = 'UAS'



    commands = str()
    rabbit = object
    currLat = float(0.000000000)
    currLon = float(0.000000000)
    destLat = float(0.000000000)
    destLon = float(0.000000000)
    currBearing = float(0.000)
    destBearing = float(0.000)
    waypoint = int()
    moduleName = "rabbit"
    ID = 2
    defaultAlt = float(5)


    def startRabbit(self):

        creds = pika.PlainCredentials(username=self.username, password=self.password)
        try:

            self.connection = pika.BlockingConnection(pika.ConnectionParameters(\
                host = self.host,\
                virtual_host = self.vhost,\
                credentials = creds,
                heartbeat_interval = 0))
            self.channel = self.connection.channel()

            self.channel.exchange_declare(exchange = self.exchange, \
                                     type = 'topic')
            print " [x] %s:%s" % (self.moduleName,"Consumer connected to rabbit server")
        except:
            print " [x] %s:%s" % (self.moduleName,"Consumer failed to connect to rabbit server")

        try:
            self.connectionPublish = pika.BlockingConnection(pika.ConnectionParameters(\
                host = self.host,\
                virtual_host = self.vhost,\
                credentials = creds,
                heartbeat_interval=0))

            self.publishChannel = self.connectionPublish.channel()

            self.publishChannel.exchange_declare(exchange = self.exchange, \
                                 type = 'topic')
            print " [x] %s:%s" % (self.moduleName,"Producer connected to rabbit server")
        except:
            print " [x] %s:%s" % (self.moduleName,"Producer failed to connect to rabbit server")

        try:
            result = self.channel.queue_declare(exclusive = True)
            queue_name = result.method.queue

            binding_keys = ['Autopilot.commands', 'Waypoint.track', 'mavproxy']
            i = 0
            while i<len(binding_keys):
                self.channel.queue_bind(exchange = self.exchange, \
                       queue = queue_name, \
                       routing_key = binding_keys[i])
                i=i+1

            self.channel.basic_consume(callback, \
                          queue = queue_name, \
                          no_ack = True)
            print " [x] %s:%s" % (self.moduleName,"Callback initialized")
        except:
            print " [x] %s:%s" % (self.moduleName,"Failed to initialize callback")




class Threaded(object):
    def __init__(self, function, *args, **kwargs):
        print "The threaded object initalized"
        self.thread = None
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        #self.setDaemon = True


    def start(self):
        self.thread = threading.Thread(target = self.function)
        self.thread.daemon = True
        self.thread.start()


    def stop(self):
            print "it stopped"
            #Vars.connection.close()
            self.thread.join()
            #Vars.enough = True
            #sys.exit(0)
            #self.thread.join()



def callback(ch, method, properties, body):
    try:
        input = body.split(",")
        if method.routing_key == "Autopilot.commands":
            #TODO: Add takeoff callback function
            print "[x] %s: %s: %s" % (str(time.time()), method.routing_key, str(body))
            if len(input)>0:
                if input[0] == "Loiter":
                    if len(input) == 4:
                        args = [str(input[1]), str(input[3]), str(input[3])]
                        sendGuidedPoint(args)
                        #print args
                    else:
                        print "No actionable commands in Loiter packet"
                elif input[0] == "Land":
                    sendLand("hi")

                elif input[0] == "Move":
                    if len(input) == 4:
                        args = [float(input[1]), float(input[2]), float(input[3])]
                        sendMove(args)
                        #print args
                    else:
                        print "No actionable commands in Move packet"
                elif input[0] == "TrackRestore":
                    if len(input)==2:
                        args = [int(input[1])]
                        sendJump(args)
                    else:
                        print "No actionable commands in Jump packet"

                elif input[0] == "Launch":
                    if int(input[1]) is vars.ID:
                        print "This command was meant for me"
                        print input
                        args = [vars.defaultAlt] #,str(input[2]),str(input[3])]
                        sendTakeoff(args)

                    else:
                        print "this command was meant for the other copter"
                        print input

                elif input[0] == "guide":
                    if int(input[1]) is vars.ID:
                        print "This command was meant for me"
                        print input
                        args = [str(input[2]),str(input[3]),vars.defaultAlt]
                        sendGuidedPoint(args)

                    else:
                        print "this command was meant for the other copter"
                        print input


            else:
                print "Empty message received from rabbit"
        elif method.routing_key == "Waypoint.track":

            if input[0] == "TrackRestore":
                    print "[x] %s: %s: %s" % (str(time.time()), method.routing_key, str(body))
                    if len(input)==2:
                        args = [int(input[1])]
                        sendJump(args)
                    else:
                        print "No actionable commands in Jump packet"



    except KeyboardInterrupt:
        consume.stop()
    except:
        print "it failed here"
        traceback.print_exc()

def startConsuming():
    try:

        Vars.channel.start_consuming()
        print " [x] %s:%s" % (Vars.moduleName,"Consumer started")
    except:
        print " [x] %s:%s" % (Vars.moduleName,"Failed to start consuming")

def sendGuidedPoint(args):
    print "sending a guided point yay!!!!"
    Vars.rabbit.sendGuided(args)

def sendLand(args):
    Vars.rabbit.land(args)

def sendMove(args):
    Vars.rabbit.move(args)

def sendTakeoff(args):
    Vars.rabbit.cmd_takeoff(args)


def sendJump(args):
    Vars.rabbit.jump(args)

class Rabbit(mp_module.MPModule):

    def __init__(self, mpstate):
        super(Rabbit, self).__init__(mpstate, "rabbit", "Rabbit commands", True)
        self.add_command('Loiter', self.sendGuided, "send guided command")
        self.add_command('Land', self.land, "Lands the copter at its current location")
        self.add_command('Move', self.move, "Moves to a point: given dist(meters) bearing alt(meters")
        self.add_command('jump', self.jump, "jumps to a waypoint")
        #print "command added"
        Vars.startRabbit()
        consume.start()



    def jump(self, args):
        if len(args) == 1:
            seq = int(args[0])
            self.master.mav.mission_set_current_send(self.status.target_system, self.status.target_component, seq)
            print "Changing current waypoint to " + str(seq)
        else:
            print "Usage: jump waypoint#"

    def move(self, args):
        if len(args) == 3:
            dist = float(args[0])
            bearing = float(args[1])
            alt = float(args[2])
            #if (bearing >=0 and bearing<=360):
            #print "You did something right!"
            print "Current Lat: " + str(Vars.currLat) + " Current Lon: " + str(Vars.currLon)

            Vars.destLat, Vars.destLon = newCoordinates(dist, bearing)

            print "Lat: " + str(Vars.destLat) + " Lon: " + str(Vars.destLon)
            self.master.mav.mission_item_send (self.status.target_system,
                                           self.status.target_component,
                                           0,
                                           0,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                           2, 0, 1, 0, 0, 0,
                                           Vars.destLat, Vars.destLon, alt)
            #self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4) #4 for copters, #8 for planes, #6 for rover
        #else:
               # print "Invalid bearing: between 0 and 360"
        else:
            print "Incorrect Arguments: dist(meters) bearing alt(meters)"


    def land(self, args):
        self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)

    def sendGuided(self, args):
        try:
            print "arguments in sendGuided: " + str(args)
            if len(args) == 3:
                lat = float(args[0])
                lon = float(args[1])
                alt = float(args[2])
                print str(lat) + "," + str(lon) + "," + str(alt)
                self.master.mav.mission_item_send(self.settings.target_system,
                                                  self.settings.target_component,
                                                  0,
                                                  self.module('wp').get_default_frame(),
                                                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                  2, 0, 0, 0, 0, 0,
                                                  lat, lon, alt)
                self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4) #4 for copters, #8 for planes
            else:
                print "Failed to enter arguments correctly: Usage: Loiter lat lon alt"
                return
        except:
            return

    def cmd_takeoff(self, args):
        try:
            '''take off'''
            mode_mapping = self.master.mode_mapping()
            mode = 'GUIDED'
            if mode not in mode_mapping:
                print "oops"
                return
            else:
                modenum = mode_mapping[mode]



            if (len(args) < 1):
                print("Usage: takeoff ALTITUDE_IN_METERS")
                return

            if (len(args) == 1):
                altitude = float(args[0])
                print "Takeoff Altitude: " + str(altitude)
                print("Take Off started")
                #argsOut = [str(args[1]), str(args[2]), altitude]
                self.master.set_mode(modenum)
                self.master.arducopter_arm()
                self.master.mav.command_long_send(
                    self.settings.target_system,  # target_system
                    mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL,  # target_component
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
                    0,  # confirmation
                    0,  # param1
                    0,  # param2
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    altitude)  # param7
                #sendGuidedPoint(argsOut)
        except:
            traceback.print_exc()
            return




    #def mavlink_packet(self, m):
        '''
        #handle a mavlink packet
        mtype = m.get_type()
        #print m
        if mtype=="HEARTBEAT":
            #print m
            raw = str(m).split(",")
            i=0
            while(i<len(raw)):
                raw[i] = raw[i].strip()
                #print raw[i]
                i=i+1
            mode = raw[3].split(":")[1]
            publishSomething("mode," + mode.strip(), "Direction.data")
        if mtype == 'waypoint 15':
            #print m
            hi = m
            #placeholder: only sent from the autopilot when the waypoint is changed

        if mtype=="RC_CHANNELS_RAW":
            raw = str(m).split(",")
            i=0
            while(i<len(raw)):
                raw[i] = raw[i].strip()
                #print raw[i]
                i=i+1

            chan3in= raw[5].split(":")[1]
            publishSomething("chan3in," + str(chan3in), "Direction.data")

        if (mtype=="SERVO_OUTPUT_RAW"):

            raw = str(m).split(",")

            i=0
            while(i<len(raw)):
                raw[i] = raw[i].strip()
                #print raw[i]
                i=i+1

            chan1out= raw[2].split(":")[1]
            publishSomething("chan1out," + str(chan1out), "Direction.data")

        if mtype == 'MISSION_CURRENT':
            #print m
            input = str(m).split(':')
            data = str(input[1].strip("}"))
            data = str(data.strip(" "))
            if(int(data) != Vars.waypoint):
                Vars.waypoint = int(data)
                print data
            publishSomething("TrackChange," + str(data), "Waypoint.track")

            #hi = m
            #placeholder: Continuously pushed the waypoint it's going to
        if mtype == 'GPS_RAW_INT':
            #print m
            input = str(m).split(',')
            i=0
            while i<len(input):
                input[i] = input[i].strip()
                i=i+1
            if len(input)>0:
                gpsdata = [input[2], input[3]]
                i=0
                while i<len(gpsdata):
                    splitstring = gpsdata[i].split(" : ")
                    gpsdata[i] = splitstring[1]
                    i=i+1

                #print "Lat: " + str(gpsdata[0]) + "   Lon: " + str(gpsdata[1])
                Vars.currLat = gpsdata[0]
                Vars.currLon = gpsdata[1]
                #print "I am here"

                outPixLat= int(Vars.currLat)
                outPixLon = int(Vars.currLon)
                #print "I got to the end of the here"

                lat = float(int(Vars.currLat)/10000000.00000000000)
                lon = float(int(Vars.currLon)/10000000.00000000000)
                #print "lat: " + str(lat) + "   lon: " + str(lon)
                pixLat = float(int(outPixLat)/10000000.00000000000)
                pixLon = float(int(outPixLon)/10000000.00000000000)
                #print "pixLat: " + str(pixLat) + "     pixLon: " + str(pixLon)
                t = time.localtime()
                out = str(t[0]) + "-" + str(t[1]) + "-" + str(t[2]) + " " + str(t[3]) + ":" + str(t[4]) + ":" + str(t[5]) + "." + (str(time.time()).split("."))[1] + "," + str(pixLat) + "," + str(pixLon) + "," + str(0)
                #print out
                out2 = str(time.time()) + "," + str(lat) + "," + str(lon) + "," + str(0)
                
                #print out2

                publishSomething(out, "GPS_data")
                publishSomething(out2, "GPS1")
                publishSomething(out2, "GPS2")
                #publishSomething("Lat: " + str(gpsdata[0]) + "   Lon: " + str(gpsdata[1]), "Hello.out")
                

            else:
                print "NoGPS data in packet, something went realllyyy wrong"
        #elif mtype == "SYS_STATUS":
            #print m
            '''



def toRadians(number):                                                  #to radians normally, because I like methods
    numout = float()
    numout = float(math.radians(number))
    return numout

def newCoordinates(distance, bearing):                             #calculates the new coordinates based on current position, bearing, and distance to move
      lat = float(int(Vars.currLat)/10000000.00000000000)
      #print "Current lat: " + str(lat)
      lon = float(int(Vars.currLon)/10000000.00000000000)
      #print "Current lon: " + str(lon)

      current = geopy.Point(lat, lon)
      destination = VincentyDistance(meters=distance).destination(current, bearing)
      lat2, lon2 = destination.latitude, destination.longitude

      '''latOut = int((lat2*10000000))                                                     #find a better way to do this that works for larger values
      print "Latitude converted: " + str(latOut)
      lonOut = int((lon2*10000000))                                                     #find a better way to do this that works for larger values
      print "Longitude converted: " + str(lonOut)'''


      return lat2, lon2

def bearingToPoint(destLat, destLon):                                           #calculates bearing based on current position and next position

      destLat = toRadians(destLat)
      destLon = toRadians(destLon)
      currLat = toRadians(Vars.currLat)
      currLon = toRadians(Vars.currLon)
      dLon = destLon - currLon
      y = math.sin(dLon) * math.cos(destLat)
      x = math.cos(currLat)*math.sin(destLat) -math.sin(currLat)*math.cos(destLat)*math.cos(dLon)
      brng = math.atan2(y, x)* (180/math.pi)
      brng = (brng)%360

      return brng


'''def distanceToCoordinates():                                              #Gives the distance from one GPS point to another using GEOPY
    current = geopy.Point(lat, lon)
    dest = geopy.Point(self.destLat, self.destLon)
    relation = vincenty(current, dest).meters
    self.distanceToPoint = relation'''





def publishSomething(body, key):
    try:
        Vars.publishChannel.basic_publish(exchange=Vars.exchange, \
        routing_key=key,\
        body=body)

    except:
        print " [x] %s:%s" % (Vars.moduleName,"Failed to publish to the rabbit server")


Vars = vars()
consume = Threaded(startConsuming)


def init(mpstate):
    Vars.rabbit = Rabbit(mpstate)
    '''initialise module'''
    return Vars.rabbit
