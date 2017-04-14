from tkinter import Tk, Label, Button
import pika
import traceback
import os

username = 'UAS'
password = 'UAS'
host = '127.0.0.1'
vhost = 'UASHost'
exchange = 'UAS'
creds = pika.PlainCredentials(username=username, password=password)

try:
    connectionPublish = pika.BlockingConnection(pika.ConnectionParameters( \
        host=host, \
        virtual_host=vhost, \
        credentials=creds,
        heartbeat_interval=0))

    publishChannel = connectionPublish.channel()

    publishChannel.exchange_declare(exchange=exchange, \
                                         type='topic')
    print " [x] %s" % ("Producer connected to rabbit server")
except:
    traceback.print_exc()
    print " [x] %s" % ( "Producer failed to connect to rabbit server")


class MyFirstGUI:
    def __init__(self, master):
        self.master = master
        master.title("Fly?")

        self.label = Label(master, text="Initiate auto flight")
        self.label.pack()

        self.greet_button = Button(master, text="Takeoff", command=self.greet)
        self.greet_button.pack()

        self.greet_button = Button(master, text="Fly", command=self.guide)
        self.greet_button.pack()

        self.close_button = Button(master, text="Close", command=self.exit)
        self.close_button.pack()

    def greet(self):
        body = "startFlight"
        try:
            publishChannel.basic_publish(exchange=exchange, \
                                              routing_key="interface.launch", \
                                              body=body)
            print "Publishing"
        except:
            traceback.print_exc()
            print " [x] %s" % ("Failed to publish to the rabbit server")

    def exit(self):
        os._exit(0)

    def guide(self):
        body = "guide"
        try:
            publishChannel.basic_publish(exchange=exchange, \
                                         routing_key="interface.launch", \
                                         body=body)
            print "Publishing"
        except:
            traceback.print_exc()
            print " [x] %s" % ("Failed to publish to the rabbit server")


try:
    root = Tk()
    my_gui = MyFirstGUI(root)
    root.mainloop()
except KeyboardInterrupt:
    print "Program exicted"
except:
    print "Program exited"
    traceback.print_exc()
finally:
    os._exit(0)