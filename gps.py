from serial import Serial
from math import *
import time

ser = Serial('/dev/ttyUSB0', 4800, timeout=5)
initPos_x = 0
initPos_y = 0
initialPosition = (0, 0)
currentPosition = (0, 0)


def dms_to_dec(x):
    ''' Returns the dec form of [x]. Requires: [x] is in the form of ####.#### '''
    minutes = x % 100
    degrees = float(int(x / 100))
    return degrees + minutes / 60


def get_latlong():
    ''' Uses the Serial Port to get latlong of position using BU-353S4'''
    line = ser.readline()
    line = line.decode('utf-8')  # Convert bytes to str
    splitline = line.split(',')
    if(splitline[0] == '$GPGGA'):
        print(line)
        latitude = str(dms_to_dec(float(splitline[2])))
        latDirec = splitline[3]
        longtitude = str(dms_to_dec(float(splitline[4])))
        longDirec = splitline[5]
        if(longDirec == 'W'):
            longtitude = '-' + longtitude
        return (latitude, longtitude)
    # Following is how to use in unix env.
    # https://www.egr.msu.edu/classes/ece480/capstone/spring15/group14/uploads/4/2/0/3/42036453/wilsonappnote.pdf

    # May need to update driver, and install GPSInfo utility

    # $GPZDA = Date and Time ? Hours 01, Minutes 22, Seconds 14.000, Day 4th, Month December, Year 2017, No_local_timezone_set, Checksum
    # $GPRMC = Recommended minimum data for GPS

    # ******************************************************
    # $GPGGA = 3D Fix Information ? Time of fix, Latitude, North or South, Longitude, East or West, Fix type, Number of satellites being tracked, Accuracy, Altitude above mean sea level, Meters, Height of mean sea level above WGS84, Meters, empty, empty, checksum
    # $GPGGA,220314.000,4227.0549,N,07629.6206,W,1,10,1.1,159.2,M,-33.9,M,,0000*61
    # ******************************************************

    # $GPGSA = Type of GPS fix and Number of satellites used ? Auto select 2D/3D or manual, 1=no-fix 2=2D-fix 3=3D-fix, ? , ? , ? upto 12 satellites being used ? , precision, Horizontal precision, Vertical precision, Checksum
    # $GPGSV = Detailed Satellite Data
    # $GPTXT = Custom data sent from the GPS unit. Can be anything the creator wanted such as version numbers, antenna status

    # For more info: http://www.pi-resource.com/?page_id=341

# https://epsg.io/transform
# Apparently, this is in terms of meters
# It says that it's not as useful for distance compared to other projections
# I can try others? This seems to work as of now.


def latlong_to_WebMerc(lat, lon):
    ''' Convert lat long to UTM (EPSG:3857). Requires: [lat lon] in dec form. '''
    x = lon * 20037508.34 / 180.0
    y = (log(tan((90 + lat) * pi / 360)) / (pi / 180)) * 20037508.34 / 180
    print("x is " + str(x) + " and y is " + str(y))
    return (x, y)


def WebMerc_to_latlong(x, y):
    ''' Convert Web Mercator to latitude, longitude. '''
    lon = x * 180 / 20037508.34
    lat = atan(e ** ((y * 180 / 20037508.34) * (pi / 180))) * 360 / pi - 90
    print("lat is " + str(lat) + " and long is " + str(lon))
    return (lat, lon)

# Something is wrong with the distance... The values produced are not so accurate...


def get_distance(x1, y1, x2, y2):
    ''' Returns the distance between two points of (x1, y1) and (x2, y2).'''
    x2 = abs(x2)
    x1 = abs(x1)
    y2 = abs(y2)
    y1 = abs(y1)
    dist = sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    print("We are " + str(dist) + " meters away from our initial position.\n")
    return dist


def main():
    # Grab Global Variables
    global initialPosition
    global currentPosition
    global initPos_x
    global initPos_y
    latlong = None
    while(latlong == None):  # Ensure valid latlong values
        latlong = get_latlong()

    lat = float(latlong[0])
    lon = float(latlong[1])
    # # Convert to Web Mercator x and y coordinates
    xy = latlong_to_WebMerc(lat, lon)
    # xy = latlong_to_WebMerc(42.4476945, -76.4838055)
    # Results:
    # x = -8514138.28
    # y = 5228279.61
    x = xy[0]
    y = xy[1]
    # Convert back to lat and long
    latlong2 = WebMerc_to_latlong(x, y)
    lat2 = latlong2[0]
    long2 = latlong2[1]

    if(initialPosition == (0, 0)):  # Assumes we never go to the center of Earth
        initPos_x = x  # Updates the initial position, only once.
        initPos_y = y
        initialPosition = xy
        print(initialPosition)
    currentPosition = xy  # Updates the current position
    distance_from_init = get_distance(x, y, initPos_x, initPos_y)


if __name__ == "__main__":
    while(1):
        main()
        time.sleep(0.5)

# while(1):
#     get_latlong()
#     time.sleep(0.5)

# print(dms_to_dec(4227.0992))
# print(dms_to_dec(07629.6243))
# print(dms_to_dec(4227.0750))
# print(dms_to_dec(07629.6257))
# We're at 4227.0992 latitude and 07629.6243 longtitude
# We're at 4227.0750 latitude and 07629.6257 longtitude
