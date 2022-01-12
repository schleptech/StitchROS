#!/usr/bin/env python3
import os
import pyzed.sl as sl
import rosnode
import time
import json
import AWSIoTPythonSDK.MQTTLib as AWSIoTPyMQTT
import rosnode
import glob
import shutil
import Jetson.GPIO
ENDPOINT = "a1ob5hw097inad-ats.iot.us-east-2.amazonaws.com"
CLIENT_ID = "Stitch0001"
PATH_TO_CERTIFICATE = "9173cca5aadbf7f461f09e8794add769a67d96c37df65da280ebf6ae30a4b9d4-certificate.pem.crt"
PATH_TO_PRIVATE_KEY = "9173cca5aadbf7f461f09e8794add769a67d96c37df65da280ebf6ae30a4b9d4-private.pem.key"
PATH_TO_AMAZON_ROOT_CA_1 = "AmazonRootCA1.pem"
MESSAGE = "Hello World"
TOPIC = "stitchData"
myAWSIoTMQTTClient = AWSIoTPyMQTT.AWSIoTMQTTClient(CLIENT_ID)
topics = []

def logStitch(msg):
    f = open("StitchLog.txt", "a")
    f.write(msg + '\n')
    f.close()

def testCellConnection():
    if os.system("ping -c 4 " + os.environ.get('CELL')) == 0: return True 
    else: 
        logStitch('Could Not Connect')
        return False

def testOusterConnection():
    if os.system("ping -c 4 " + os.environ.get('OUSTER')) == 0: return True
    else: 
        logStitch('No Response From Ouster')
        return False

def testZedConnection():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.sdk_verbose = False

    # Open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS: return True 
    else: 
        logStitch('No Response From Camera')
        return False
        

def initTest():
    return testCellConnection() and testOusterConnection and testZedConnection


    

def stdMessage():
    diskInfo = shutil.disk_usage('/')
    try:
        topics = str(rosnode.get_node_names())
    except:
        topics = ['No Topics being Published']
    message = {
        'Published Topics' : topics,
        'Bag Files' : glob.glob('bag_*'),
        'Disk Statistics' : {
            'Used' : diskInfo[1],
            'Free' : diskInfo[2],
            'Percentage Free' : str((diskInfo[2]/diskInfo[0]) * 100) + ' %',
            'Total' : diskInfo[0]
        }
    }
    myAWSIoTMQTTClient.publish(TOPIC, json.dumps(message), 0) 

def checkTest():
    PCPub = False
    ZedPub = False
    Rec = False
    for item in rosnode.get_node_names():
        if (item == '/pub_pcl'): PCPub = True
        elif (item == '/zed2i/zed2i_node'): ZedPub = True
        elif (item[:7] == '/record'): Rec = True
    if(not (testCellConnection() and PCPub and ZedPub and Rec)): 
        Jetson.GPIO.setup(22, Jetson.GPIO.OUT, initial=Jetson.GPIO.LOW)
    else: Jetson.GPIO.setup(22, Jetson.GPIO.OUT, initial=Jetson.GPIO.HIGH)
    


if __name__ == '__main__':
    f = open('StitchLog.txt', 'w')
    f.close()
    Jetson.GPIO.setmode(Jetson.GPIO.BOARD)
    Jetson.GPIO.setup(22, Jetson.GPIO.OUT, initial=Jetson.GPIO.LOW)
    while(not initTest()): 
        logStitch('Test Failed, Restarting')
        time.sleep(4)
    myAWSIoTMQTTClient.configureEndpoint(ENDPOINT, 8883)
    myAWSIoTMQTTClient.configureCredentials(PATH_TO_AMAZON_ROOT_CA_1, PATH_TO_PRIVATE_KEY, PATH_TO_CERTIFICATE)
    myAWSIoTMQTTClient.connect()
    myAWSIoTMQTTClient.publish(TOPIC, json.dumps({'Message' : 'STITCH0001 Successfuly Initiated'}), 0) 
    while(True):
        time.sleep(60)
        stdMessage()


    
