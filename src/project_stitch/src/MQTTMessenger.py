#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

import time as t
import json
import AWSIoTPythonSDK.MQTTLib as AWSIoTPyMQTT
import rosnode
import glob
import shutil
# Define ENDPOINT, CLIENT_ID, PATH_TO_CERTIFICATE, PATH_TO_PRIVATE_KEY, PATH_TO_AMAZON_ROOT_CA_1, MESSAGE, TOPIC, and RANGE
ENDPOINT = "a1ob5hw097inad-ats.iot.us-east-2.amazonaws.com"
CLIENT_ID = "Stitch0001"
PATH_TO_CERTIFICATE = "9173cca5aadbf7f461f09e8794add769a67d96c37df65da280ebf6ae30a4b9d4-certificate.pem.crt"
PATH_TO_PRIVATE_KEY = "9173cca5aadbf7f461f09e8794add769a67d96c37df65da280ebf6ae30a4b9d4-private.pem.key"
PATH_TO_AMAZON_ROOT_CA_1 = "AmazonRootCA1.pem"
MESSAGE = "Hello World"
TOPIC = "stitchData/points"


myAWSIoTMQTTClient = AWSIoTPyMQTT.AWSIoTMQTTClient(CLIENT_ID)
myAWSIoTMQTTClient.configureEndpoint(ENDPOINT, 8883)
myAWSIoTMQTTClient.configureCredentials(PATH_TO_AMAZON_ROOT_CA_1, PATH_TO_PRIVATE_KEY, PATH_TO_CERTIFICATE)
myAWSIoTMQTTClient.connect()
#message = {"message" : "Point Cloud Publishing at " + str(t.time())}
diskInfo = shutil.disk_usage('/')
try:
    topics = str(rosnode.get_node_names())
except:
    topics = 'No Topics being Published'
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
    
myAWSIoTMQTTClient.disconnect()
