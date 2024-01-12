import paho.mqtt.client as mqtt
import binascii
import struct
from serial import Serial

def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))
    mqttc.subscribe("controller/stick", qos=0)

def on_message(mqttc, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
    decrypt_message(msg.payload)

def decrypt_message(encrypted_msg):
    [lx, rx, ry, ly, b1, b2, b3] = struct.unpack('ffffcce', encrypted_msg)
    print(f"Decrypted Message: {[lx, rx, ry, ly]}")

    print('{0:08b}'.format(int(binascii.hexlify(b1), 16)))
    print('{0:08b}'.format(int(binascii.hexlify(b1), 16)))
    print("---")

mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect

mqttc.connect("192.168.12.1", 1883, 60)
mqttc.loop_forever()
