import paho.mqtt.client as mqtt_client #import the client1
import time
############

from_KMR = "--"
to_KMR = "softplc,3"
status_KMR = "--"
from_ABB ="--"
to_ABB = "--"
status_ABB = "--"
btn_status = "STOP"
start_request = False
system_rediness = False
KMR_rediness = False
ABB_rediness = False
thinker_feedback = "--"
system_run = False
cmd_to_KMR = "--"
cmd_to_ABB = "--"
previous_to_KMR = ""

def on_connect(client, userdata, flags, rc):  # The callback for when the client connects to the broker
    print("Connected with result code {0}".format(str(rc)))  # Print result of connection attempt
    client.subscribe("uncasing_station/UR10/joint_angles")  # Subscribe to the topic “digitest/test1”, receive any messages published on it
    print ()


def on_message(client, userdata, msg):  # The callback for when a PUBLISH message is received from the server.
    global btn_status
    print("Message received-> " + msg.topic + " " + str(msg.payload))  # Print a received msg
    btn_status = (msg.payload).decode('utf-8')

def socket_client_mqtt_sub_conn():
    global from_abb_emulator
    client = mqtt_client.Client("digi_mqtt_test")  # Create instance of client with client ID “digi_mqtt_test”
    client.on_connect = on_connect  # Define callback function for successful connection
    client.on_message = on_message  # Define callback function for receipt of a message
    # client.connect("m2m.eclipse.org", 1883, 60)  # Connect to (broker, port, keepalive-time)
    client.connect("192.168.68.116",11883)
    client.loop_forever()  # Start networking daemon

socket_client_mqtt_sub_conn()