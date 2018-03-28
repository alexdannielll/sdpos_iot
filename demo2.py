#!/usr/bin/env python
# -*- coding: cp1252 -*-

#Libraries
import time
import json
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from gpiozero import Buzzer

#set GPIO Pin buzzer
buzzer = Buzzer(17)

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins Sensor
GPIO_TRIGGER = 18
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def on_connect(mosq, obj, rc):
	print("rc: " + str(rc))

def on_message(mosq, obj, msg):
	global message
	print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
	message = msg.payload

def on_publish(mosq, obj, mid):
	print("mid: " + str(mid))

def on_log(mosq, obj, level, string):
	print(string)
	
def distance():
	# set Trigger to HIGH
	GPIO.output(GPIO_TRIGGER, True)
 
	# set Trigger after 0.01ms to LOW
	time.sleep(0.00001)
	GPIO.output(GPIO_TRIGGER, False)
 
	StartTime = time.time()
	StopTime = time.time()
 
	# save StartTime
	while GPIO.input(GPIO_ECHO) == 0:
		StartTime = time.time()
 
	# save time of arrival
	while GPIO.input(GPIO_ECHO) == 1:
		StopTime = time.time()
 
	# time difference between start and arrival
	TimeElapsed = StopTime - StartTime
	# multiply with the sonic speed (34300 cm/s)
	# and divide by 2, because there and back
	distance = (TimeElapsed * 34300) / 2
 
	return distance
	
# topicos providos por este sensor
topic = "/admin/6ad7/attrs"

# cria um identificador baseado no id do sensor
client = mqtt.Client()

client.on_log = on_log

client.username_pw_set("USER", "PASSWORD")

# conecta no broker
client.connect("18.231.32.58", 1883, 60)

if __name__ == '__main__':
	try:
		while True:
			dist = distance()
			print ("Measured Distance = %.1f cm" % dist)
			
			# codificando o payload como big endian, 2 bytes
			payload = json.dumps({"dist": dist})
			
			# envia a publicação
			(rc, mid) = client.publish(topic, payload, qos=1)
			
			print topic + "/" + str(dist)
			
			client.on_message = on_message
			client.on_connect = on_connect
			client.on_publish = on_publish
			
			if dist < 20.0:
				buzzer.on()
				
			time.sleep(1)
			
			if dist < 20.0:
				buzzer.off()
		
		client.disconnect()
		
		# Reset by pressing CTRL + C
	except KeyboardInterrupt:
		print("Measurement stopped by User")
		GPIO.cleanup()