import RPi.GPIO as GPIO
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import message_filters
from adafruit_servokit import ServoKit
from gps import *
import time, inspect
import subprocess
import socket
from py_master.armLib import Arm_Device
import sqlite3
import Adafruit_DHT
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
sensor = Adafruit_DHT.DHT11
gpioTemp = 25

Arm = Arm_Device()

# Set up servo motor kit
kit = ServoKit(channels=16)
front_right = 0
back_right = 1
front_left = 2
back_left = 3
# Define the channels for each steer motor
front_steer_motor = 5
back_steer_motor = 4

# define default angle position for steer motors
kit.servo[front_steer_motor].angle = 95
kit.servo[back_steer_motor].angle = 95

# Define motor speeds
stop_speed = 90
move_speed = 100
reverse_speed = 68

class masTer(Node):
    def __init__(self):
        super().__init__('tes_Ter')
        
        # Declare distance_msg and camera_msg as global variables
        global distance_msg, camera_msg
        distance_msg = None
        camera_msg = None
        
        # Create message filters for distance and camera topics
        distance_sub = self.create_subscription(String, 'distance', self.distance_callback, 2)
        camera_sub = self.create_subscription(String, 'camera', self.camera_callback, 1)
        distance_sub
        camera_sub
        
    def distance_callback(self, msg):
        global distance_msg
        distance_msg = msg
        
    def camera_callback(self, msg):
        global camera_msg
        camera_msg = msg

def moveForward():
    kit.servo[front_right].angle = move_speed
    kit.servo[front_left].angle = move_speed
    kit.servo[back_right].angle = move_speed
    kit.servo[back_left].angle = move_speed
    
def moveForwardSlow():
    kit.servo[front_right].angle = 94
    kit.servo[front_left].angle = 94
    kit.servo[back_right].angle = 94
    kit.servo[back_left].angle = 94

def moveBack():
    kit.servo[front_right].angle = reverse_speed
    kit.servo[front_left].angle = reverse_speed
    kit.servo[back_right].angle = reverse_speed
    kit.servo[back_left].angle = reverse_speed
    
def moveBackSlow():
    kit.servo[front_right].angle = 84
    kit.servo[front_left].angle = 84
    kit.servo[back_right].angle = 84
    kit.servo[back_left].angle = 84
    
def moveLeft():
    kit.servo[front_right].angle =120
    kit.servo[front_left].angle = 50
    kit.servo[back_right].angle = 120
    kit.servo[back_left].angle = 50

def moveLeftSlow():
    kit.servo[front_right].angle =100
    kit.servo[front_left].angle = 75
    kit.servo[back_right].angle = 100
    kit.servo[back_left].angle = 75
    
def moveRight():
    kit.servo[front_right].angle =50
    kit.servo[front_left].angle = 120
    kit.servo[back_right].angle = 50
    kit.servo[back_left].angle = 120

def moveRightSlow():
    kit.servo[front_right].angle =75
    kit.servo[front_left].angle = 100
    kit.servo[back_right].angle = 75
    kit.servo[back_left].angle = 100

#For pivot left and right, dont go over 60 or 130 in values
def pivotLeft():
    kit.servo[front_steer_motor].angle = 60
    kit.servo[back_steer_motor].angle = 130
    
def pivotLeftSlow():
    kit.servo[front_steer_motor].angle = 60
    kit.servo[back_steer_motor].angle = 130

def pivotRight():
    kit.servo[front_steer_motor].angle = 130
    kit.servo[back_steer_motor].angle = 60
    
def pivotRightSlow():
    kit.servo[front_steer_motor].angle = 130
    kit.servo[back_steer_motor].angle = 60
    
def pivotCenter():
    kit.servo[front_steer_motor].angle = 95
    kit.servo[back_steer_motor].angle = 95
    
def stop():
    kit.servo[front_right].angle = stop_speed
    kit.servo[front_left].angle = stop_speed
    kit.servo[back_right].angle = stop_speed
    kit.servo[back_left].angle = stop_speed  

def initialState():
    Arm.Arm_serial_servo_write(6, 0, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(5, 90, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(4, 90, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(3, 90, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(2, 90, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(1, 85, 1000)
    
def pickupState():
    Arm.Arm_serial_servo_write(1, 85, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(2, 10, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(3, 50, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(4, 90, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(6, 20, 1000)
    time.sleep(1)
    Arm.Arm_serial_servo_write(6, 110, 1000)

def wideView():
    Arm.Arm_serial_servo_write(1, 85, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(2, 110, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(3, 10, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(4, 35, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(5, 90, 1000) 
    
def restState():
    Arm.Arm_serial_servo_write(1, 85, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(2, 110, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(3, 10, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(4, 10, 1000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(5, 90, 1000)    

def dropCube():
    Arm.Arm_serial_servo_write(1, 85, 2000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(2, 110, 2000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(3, 120, 2000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(4, 150, 2000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(5, 90, 2000)
    time.sleep(0.5)
    Arm.Arm_serial_servo_write(6, 0, 2000)
    
def addToDatabase(shape, color, temperature, humidity):
    try:
        connection = sqlite3.connect('/home/one/rostwo/src/py_master/py_master/objectsDatabase.db')
        cursor = connection.cursor()
        
        cursor.execute("INSERT INTO objects(Shape, Color, Temperature, Humidity, Timestamp) VALUES(?, ?, ?, ?, datetime('now','localtime'))", (shape, color, temperature, humidity))
        connection.commit()
        print("Successfully picked up the object and added into database")
        cursor.close()
        connection.close()
    except sqlite3.Error as error:
        print("Failed to insert data into the table", error)
    
#If you want to delete the table and reset the Id to 1 : DELETE FROM objects; 
# UPDATE SQLITE_SEQUENCE SET seq = 0 WHERE name = 'objects';
    
            
def main(args=None):
    rclpy.init(args=args)
    mas_Ter = masTer()
    turn_list = ["left", "right"]
    global detected
    detected = 0
    global temperatureStr
    temperatureStr = ""
    global humidityStr
    humidityStr = ""
    initialState()
    time.sleep(1)
    restState()
    time.sleep(1)
    
    # Run infinite loop to receive messages
    while True:
        rclpy.spin_once(mas_Ter, timeout_sec=0)
        
        # Check for new messages and print them
        global distance_msg, camera_msg
        if (distance_msg is not None and detected == 0):
            distanceString = distance_msg.data
            distanceValue = int(distanceString)
            
            if (distanceValue < 50):
                random.shuffle(turn_list)
                turn = random.choice(turn_list)
                
                if turn == "left":
                    moveBack()
                    time.sleep(0.5)
                    moveLeft()
                    pivotLeft()
                    time.sleep(0.5)
                    pivotCenter()
                    time.sleep(0.5)
                    distance_msg = None
                   
                elif turn == "right":
                    moveBack()
                    time.sleep(0.5)
                    moveRight()
                    pivotRight()
                    time.sleep(0.5)
                    pivotCenter()
                    time.sleep(0.5)
                    distance_msg = None
                
                else:
                    distance_msg = None
                    
            else:
                moveForward()
                distance_msg = None
                
        elif camera_msg is not None:
            cameraString = camera_msg.data
            detected = 1
            
            if cameraString == "forward":
                stop()
                moveForwardSlow()
                camera_msg = None
            elif cameraString == "left":
                pivotLeftSlow()
                moveForwardSlow()
                camera_msg = None
            elif cameraString == "right":
                pivotRightSlow()
                moveForwardSlow()
                camera_msg = None
            elif cameraString == "backwards":
                stop()
                moveBackSlow()
                camera_msg = None
            elif cameraString == "center":
                stop()
                initialState()
                pickupState()
                time.sleep(0.5)
                dropCube()
                time.sleep(0.5)
                initialState()
                restState()
                while True:
                    humidity, temperature = Adafruit_DHT.read_retry(sensor,gpioTemp)
                    if humidity is not None and temperature is not None:
                        celsiusTemp = temperature
                        fahrenheitTemp = (celsiusTemp * 1.8) + 32
                        print("Temperature in fahrenheit is" ,fahrenheitTemp)
                        print("Humidity is ", humidity, "%")
                        temperatureStr = str(fahrenheitTemp)
                        humidityStr = str(humidity)
                        break
                addToDatabase('square', 'red', temperatureStr, humidityStr)
                camera_msg = None
                detected = 0
                pivotCenter()
                time.sleep(0.5)
                      
    stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
