#!/usr/bin/python

# Import mavutil
from pymavlink import mavutil
import time
from datetime import datetime

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
#print("Waiting for heartbeat...")
#master.wait_heartbeat()
#print("Got heartbeat")

# GPS_TYPE need to be MAV
while True:
    time.sleep(0.2)
    secsperweek = 604800
    time_week_ms = int((datetime.now()-datetime(1980,1,6)).total_seconds())
    time_week = time_week_ms/secsperweek
    master.mav.gps_input_send(
        0,          #Timestamp (micros since boot or Unix epoch)
        0,          #ID of the GPS for multiple GPS inputs
        8|16|32,    #Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
        time_week_ms,  #GPS time (milliseconds from start of GPS week)
        time_week,  #GPS week number
        3,          #0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        341984220,  #Latitude (WGS84), in degrees * 1E7
        -84016690,  #Longitude (WGS84), in degrees * 1E7
        0,          #Altitude (AMSL, not WGS84), in m (positive for up)
        1,          #GPS HDOP horizontal dilution of position in m
        1,          #GPS VDOP vertical dilution of position in m
        0.0,        #GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0.0,        #GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0.0,        #GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0.05,       #GPS speed accuracy in m/s
        0.05,       #GPS horizontal accuracy in m
        0.05,       #GPS vertical accuracy in m
        7           #Number of satellites visible.
    )
    print(time_week_ms)
