#! /usr/bin/env python3

#import scipy.interpolate as spi
import sys

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
    import json
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
    for i,x_i in enumerate(xprofile):
        if x_i < x_pos:
            lowerIndex = i
            # TODO: optimize later w stop condition
    return lowerIndex

# thank you to the 1nt3rn3t
def interpolation(d, x):
    output = d[0][1] + (x - d[0][0]) * ((d[1][1] - d[0][1])/(d[1][0] - d[0][0]))
    return output

def get_target_by_position(profile, x_pos, dtype=float):
    """Get target speed by position."""
    prop_speed = 4.2
    if dtype == bool:
        kind = "previous"
    else:
        kind = "linear"
    #interp = spi.interp1d(profile[0] - vehicle.time_offset * prop_speed, profile[1],
    #                      kind=kind, fill_value="extrapolate")
    #return interp(vehicle.pos)
    # HACK fix this with real interpolation function
    print('size of profile[0]=',len(profile[0]))
    index = getFix(profile[0], x_pos)
    print('index result is ', index)
    if index >= len(profile[0])-1:
        return profile[1][-1]
    elif index <= 0:
        return profile[1][0]
    else:
        interArray = [ [ profile[0][index], profile[1][index]] , [ profile[0][index+1], profile[1][index+1] ] ]
    result = interpolation(interArray, x_pos)
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
    import json
    import os
    # TODO read from array
    target_max_headway = 1
    if not os.path.exists(circles_planner_file) or os.stat(file_path).st_size == 0:
        target_speed = 30
    else:
        speed_planner = open(circles_planner_file).read()
        speed_planner = json.loads(speed_planner.replace('\n', '').replace('   ', ' ').replace('  ', ' ').replace(' ', ', '))[0]
        print('got pos_list=',speed_planner['position'])
        import ast
        postion[0] = ast.literal_eval(speed_planner['position'])
        postion[1] = ast.literal_eval(speed_planner['speed'])
        target_speed = get_target_by_position( [ profile_0, profile_1 ], xpos )
        print('target speed is ', target_speed)

    import subprocess

#    bashCommand_speed = "source /home/circles/.bashrc && /opt/ros/melodic/bin/rosparam set SP_TARGET_SPEED {}".format( target_speed )
#    bashCommand_headway = "source /home/circles/.bashrc && /opt/ros/melodic/bin/rosparam set SP_MAX_HEADWAY {}".format( target_max_headway )
#    bashCommand_speed = "/opt/ros/melodic/bin/rosparam set SP_TARGET_SPEED {}".format( target_speed )
#    bashCommand_headway = "/opt/ros/melodic/bin/rosparam set SP_MAX_HEADWAY {}".format( target_max_headway )
#    print('bashCommand_speed=', bashCommand_speed )
#    print('bashCommand_headway=', bashCommand_headway)
#    process = subprocess.Popen(bashCommand_speed.split(), stdout=subprocess.PIPE)
#    output, error = process.communicate()
#    process = subprocess.Popen(bashCommand_headway.split(), stdout=subprocess.PIPE)
#    output, error = process.communicate()
    import rospy
    rospy.set_param('SP_TARGET_SPEED', target_speed )
    rospy.set_param('SP_MAX_HEADWAY', target_max_headway )

if __name__ == "__main__":
    # TODO: make these cmd line params but use these as defaults
    gpsfile = '/etc/libpanda.d/latest_gps'
    i24_geo_file = '/etc/libpanda.d/i24_geo.json'
    circles_planner_file = '/etc/libpanda.d/circles_speed_planner.json'
    if len(sys.argv) > 2:
        myLat = float(sys.argv[1])
        myLong = float(sys.argv[2])
        main(gpsfile, i24_geo_file, circles_planner_file, myLat, myLong)
    else:
        main(gpsfile, i24_geo_file, circles_planner_file)


