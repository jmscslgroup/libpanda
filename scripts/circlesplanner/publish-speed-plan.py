#! /usr/bin/env python3

import ast
import json
import os
import rospy
import sys
import time
from std_msgs.msg import Byte, Float64

def getGPSLocation(filename):
    """Returns lat,long as a pair. If fix is not A, then return None"""
    file = open('/etc/libpanda.d/latest_gps')
    gpsstring = file.read()
    lat = None
    long = None
    vals = gpsstring.split(',')
    if vals[-1] == 'A': 
        lat = float(vals[2])
        long = float(vals[3])
    return lat,long


def getXposFromGPS(lat,long,i24_geo_file):
    file = open(i24_geo_file)
    i24_geo = json.load(file)
    i24_index = getFix(i24_geo['latitude'], lat)
    profile = [[], []]
    profile[0] = i24_geo['latitude']
    profile[1] = i24_geo['position']
    if i24_index >= len(profile[0])-1:
        return profile[1][-1]
    elif i24_index <= 0:
        return profile[1][0]
    else:
        interArray = [ [ profile[0][i24_index], profile[1][i24_index]] , [ profile[0][i24_index+1], profile[1][i24_index+1] ] ]
    result = interpolation(interArray, lat )
    return result


# find the prev/next values over which to interpolate
def getFix(xprofile, x_pos):
    lowerIndex=0
    for i, x_i in enumerate(xprofile):
        if x_i < x_pos:
            lowerIndex = i
        else:
            break
    return lowerIndex

# thank you to the 1nt3rn3t
def interpolation(d, x):
    output = d[0][1] + (x - d[0][0]) * ((d[1][1] - d[0][1])/(d[1][0] - d[0][0]))
    return output

def get_target_by_position(profile, x_pos, pub_at, dtype=float):
    """Get target speed by position."""
    prop_speed = 4.2
    elapsed_time = time.time() - pub_at
    x_pos += prop_speed * elapsed_time
    print('size of profile[0]=',len(profile[0]))
    index = getFix(profile[0], x_pos)
    print('index result is ', index)
    if x_pos >= profile[0][-1]:
        return profile[1][-1]
    elif x_pos <= profile[0][0]:
        return profile[1][0]
    else:
        interArray = [ [ profile[0][index], profile[1][index]] , [ profile[0][index+1], profile[1][index+1] ] ]
    if dtype == float:
        result = interpolation(interArray, x_pos)
    else:
        result = profile[1][index]
    return result

def main(gpsfile, i24_geo_file, circles_planner_file, myLat=None, myLong=None ):
    lat, long = getGPSLocation(gpsfile)
    if not myLat == None:
        lat = myLat
    if not myLong == None:
        long = myLong
    print('lat,long=', lat, long)
    xpos = getXposFromGPS(lat,long,i24_geo_file)
    print('xposition=', xpos)
    pos_pub = rospy.Publisher('/xpos', Float64, queue_size=10)
    pos_pub.publish(xpos)
    
    if not os.path.exists(circles_planner_file) or os.stat(file_path).st_size == 0:
        target_speed = 30
        target_speed_200 = 30
        target_speed_500 = 30
        target_speed_1000 = 30
        target_max_headway = 0
    else:
        speed_planner = json.loads(open(circles_planner_file).read())
        pub_at = ast.literal_eval(speed_planner[0]['published_at'])
        # TODO: fix hard-coded lane number to vehicle's lane assignment
        pos_list = [ast.literal_eval(record['position']) for record in speed_planner if record['lane_num'] == 2]
        # print('got pos_list {} at time {}'.format(pos_list, pub_at))
        speed_list = [ast.literal_eval(record['target_speed']) for record in speed_planner if record['lane_num'] == 2]
        headway_list = [ast.literal_eval(record['max_headway']) for record in speed_planner if record['lane_num'] == 2]
        
        target_speed = get_target_by_position([pos_list, speed_list], xpos, pub_at, dtype=float)
        target_speed_200 = get_target_by_position([pos_list, speed_list], xpos+200, pub_at, dtype=float)
        target_speed_500 = get_target_by_position([pos_list, speed_list], xpos+500, pub_at, dtype=float)
        target_speed_1000 = get_target_by_position([pos_list, speed_list], xpos+1000, pub_at, dtype=float)
        max_headway = get_target_by_position([pos_list, headway_list], xpos, pub_at, dtype=int)
        print('Speed Planner targets: {} m/s, {} gap.'.format(target_speed, 'open' if max_headway else 'close'))

    # import subprocess

#    bashCommand_speed = "source /home/circles/.bashrc && /opt/ros/melodic/bin/rosparam set SP_TARGET_SPEED {}".format( target_speed )
#    bashCommand_headway = "source /home/circles/.bashrc && /opt/ros/melodic/bin/rosparam set SP_MAX_HEADWAY {}".format( max_headway )
#    bashCommand_speed = "/opt/ros/melodic/bin/rosparam set SP_TARGET_SPEED {}".format( target_speed )
#    bashCommand_headway = "/opt/ros/melodic/bin/rosparam set SP_MAX_HEADWAY {}".format( max_headway )
#    print('bashCommand_speed=', bashCommand_speed )
#    print('bashCommand_headway=', bashCommand_headway)
#    process = subprocess.Popen(bashCommand_speed.split(), stdout=subprocess.PIPE)
#    output, error = process.communicate()
#    process = subprocess.Popen(bashCommand_headway.split(), stdout=subprocess.PIPE)
#    output, error = process.communicate()
    rospy.set_param('SP_TARGET_SPEED', target_speed )
    rospy.set_param('SP_TARGET_SPEED_200', target_speed_200 )
    rospy.set_param('SP_TARGET_SPEED_500', target_speed_500 )
    rospy.set_param('SP_TARGET_SPEED_1000', target_speed_1000 )
    rospy.set_param('SP_MAX_HEADWAY', max_headway )

    sp_speed = rospy.Publisher('/sp/target_speed', Float64, queue_size=10)
    sp_speed_200 = rospy.Publisher('/sp/target_speed_200', Float64, queue_size=10)
    sp_speed_500 = rospy.Publisher('/sp/target_speed_500', Float64, queue_size=10)
    sp_speed_1000 = rospy.Publisher('/sp/target_speed_1000', Float64, queue_size=10)
    sp_headway = rospy.Publisher('/sp/max_headway', Byte, queue_size=10)
    sp_speed.publish(target_speed)
    sp_speed_200.publish(target_speed_200)
    sp_speed_500.publish(target_speed_500)
    sp_speed_1000.publish(target_speed_1000)
    sp_headway.publish(max_headway)

if __name__ == "__main__":
    # TODO: make these cmd line params but use these as defaults
    gpsfile = '/etc/libpanda.d/latest_gps'
    i24_geo_file = '/etc/libpanda.d/i24_geo.json'
    circles_planner_file = '/etc/libpanda.d/speed_planner.json'
    if len(sys.argv) > 2:
        myLat = float(sys.argv[1])
        myLong = float(sys.argv[2])
        main(gpsfile, i24_geo_file, circles_planner_file, myLat, myLong)
    else:
        main(gpsfile, i24_geo_file, circles_planner_file)


