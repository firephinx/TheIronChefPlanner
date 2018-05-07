#!/usr/bin/env python

import time
import signal
import sys
import boto3
import json
import decimal
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError
import calendar
import datetime
from theironchef_planner import TheIronChefPlanner

def exit_gracefully(signum, frame):
    # restore the original signal handler as otherwise evil things will happen
    # in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
    signal.signal(signal.SIGINT, original_sigint)

    try:
        if raw_input("\nAre you sure you want to quit? (y/n)> ").lower().startswith('y'):
            sys.exit(1)

    except KeyboardInterrupt:
        print("Ok ok, quitting")
        sys.exit(1)

    # restore the exit gracefully handler here    
    signal.signal(signal.SIGINT, exit_gracefully)

# Helper class to convert a DynamoDB item to JSON.
class DecimalEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, decimal.Decimal):
            if o % 1 > 0:
                return float(o)
            else:
                return int(o)
        return super(DecimalEncoder, self).default(o)

dynamodb = boto3.resource('dynamodb', region_name='us-east-1', endpoint_url="https://dynamodb.us-east-1.amazonaws.com")

table = dynamodb.Table('IronChefOrders')

def GetData():
    now = datetime.datetime.utcnow() 
    date = str(now.date())
    timestamp5minago = datetime.datetime.utcnow() - datetime.timedelta(minutes = 5)
    timestamp = str(timestamp5minago.time())
    try:
        response = table.scan(
            FilterExpression=Attr('date').eq(date) & Attr('timestamp').gt(timestamp)
        )
    except ClientError as e:
        print(e.response['Error']['Message'])
    else:
        items = response['Items']
        return items

def run_main():
    lastData = 0

    tic_planner = TheIronChefPlanner()

    tic_planner.reset_robot()

    tic_planner.home_robot()

    # tic_planner.turn_on_electromagnet()

    # tic_planner.rack_gripper_1()

    # tic_planner.home_robot()

    # tic_planner.turn_on_electromagnet()

    # tic_planner.rack_gripper_2()

    #tic_planner.cook_steak()
    while(True):
        data = GetData()
        if(len(data) > 0):
            latestcommand = data[0]
            if(latestcommand['menu_item'] == "steak"):
                print ("Desired doneness is: " + latestcommand['meat_doneness'])
                tic_planner.cook_steak(latestcommand['meat_doneness'])

        time.sleep(1)

if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    run_main()