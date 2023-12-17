import random
from paho.mqtt import client as mqtt_client

broker = '192.168.12.1'
port = 1883
topic = "bms/state"

client_id = f'subscribe-{random.randint(0, 100)}'
username = 'pi'
password = '123'

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        decoded_message = msg.payload
        print(f"Received `{decoded_message}` from `{msg.topic}` topic")


    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()import time
import paho.mqtt.client as paho

def on_connect(client, userdata, flags, rc, properties=None):
    print("CONNACK received with code %s." % rc)

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def on_message(client, userdata, msg):
    received_message = msg.payload.decode('utf-8')
    print("Received message:", received_message)

client = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
client.on_connect = on_connect
client.on_subscribe = on_subscribe
client.on_message = on_message

client.username_pw_set("pi", "123")
client.connect("192.168.12.1", 1883)

client.subscribe("bms/state", qos=0)

client.loop_forever()
