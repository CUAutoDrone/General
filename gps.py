import serial
from math import *

# ser = serial.Serial('/dev/ttyUSB0', 4800, timeout = 5)
# 
# def get_latlong():
#    line = ser.readline()
#    splitline = line.split(',')
#    if(splitline[0] == '$GPGGA'):
#       latitude = splitline[2]
#       latDirec = splitline[3]
#       longtitude = splitline[4]
#       longDirec = splitline[5]
#       print("We're at "+latitude+" latitude and "+longtitude+" longtitude")
#       return (latitude, longitude)
   # Following is how to use in unix env.
   # https://www.egr.msu.edu/classes/ece480/capstone/spring15/group14/uploads/4/2/0/3/42036453/wilsonappnote.pdf
   
   # May need to update driver, and install GPSInfo utility
   
   # $GPZDA = Date and Time ? Hours 01, Minutes 22, Seconds 14.000, Day 4th, Month December, Year 2017, No_local_timezone_set, Checksum
   # $GPRMC = Recommended minimum data for GPS
   
   # ******************************************************
   # $GPGGA = 3D Fix Information ? Time of fix, Latitude, North or South, Longitude, East or West, Fix type, Number of satellites being tracked, Accuracy, Altitude above mean sea level, Meters, Height of mean sea level above WGS84, Meters, empty, empty, checksum
   # ******************************************************
   
   # $GPGSA = Type of GPS fix and Number of satellites used ? Auto select 2D/3D or manual, 1=no-fix 2=2D-fix 3=3D-fix, ? , ? , ? upto 12 satellites being used ? , precision, Horizontal precision, Vertical precision, Checksum
   # $GPGSV = Detailed Satellite Data
   # $GPTXT = Custom data sent from the GPS unit. Can be anything the creator wanted such as version numbers, antenna status
   
   #For more info: http://www.pi-resource.com/?page_id=341
   
# https://epsg.io/transform
# Apparently, this is in terms of meters
# It says that it's not as useful for distance compared to other projections
# I can try others? This seems to work as of now. 
def latlong_to_WebMerc(lat, long):
   # convert lat long to UTM (EPSG:3857)
   x = long * 20037508.34 / 180
   y = (log(tan((90 + lat) * pi / 360)) / (pi / 180)) * 20037508.34 / 180
   print("x is " + str(x) + " and y is " + str(y))
   return (x, y)

def WebMerc_to_latlong(x, y):
   # Convert Web Mercator to latitude, longitude
   long = x * 180 / 20037508.34
   lat = atan(e ** ((y * 180 / 20037508.34) * (pi / 180))) * 360 / pi - 90
   print("lat is " + str(lat) + " and long is " + str(long))
   return (lat, long)

def main():
   # latlong = get_latlong()
   # lat = latlong[0]
   # long = latlong[1]
   # # Convert to Web Mercator x and y coordinates
   # xy = latlong_to_WebMerc(x, y)
   xy = latlong_to_WebMerc(42.4476945, -76.4838055)
   # Results:
   # x = -8514138.28
   # y = 5228279.61
   x = xy[0]
   y = xy[1]
   # Convert back to lat and long
   latlong2 = WebMerc_to_latlong(x, y)
   lat2 = latlong2[0]
   long2 = latlong2[1]



if __name__ == "__main__":
   main()

