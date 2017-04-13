from tkinter import Tk, Label, Button
import pika

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
        credentials=creds))

    publishChannel = connectionPublish.channel()

    publishChannel.exchange_declare(exchange=exchange, \
                                         type='topic')
    print " [x] %s" % ("Producer connected to rabbit server")
except:
    print " [x] %s" % ( "Producer failed to connect to rabbit server")


class MyFirstGUI:
    def __init__(self, master):
        self.master = master
        master.title("Fly?")

        self.label = Label(master, text="Initiate auto flight")
        self.label.pack()

        self.greet_button = Button(master, text="Start", command=self.greet)
        self.greet_button.pack()

        self.close_button = Button(master, text="Close", command=master.quit)
        self.close_button.pack()

    def greet(self):
        body = "startFlight"
        try:
            publishChannel.basic_publish(exchange=exchange, \
                                              routing_key="interface.launch", \
                                              body=body)
            print "Publishing"
        except:
            print " [x] %s" % ("Failed to publish to the rabbit server")

root = Tk()
my_gui = MyFirstGUI(root)
root.mainloop()