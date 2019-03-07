import serial

ser = serial.Serial('/dev/ttyUSB0', 4800, timeout = 5)

while(1):
   line = ser.readline()
   splitline = line.split(',')
   if(splitline[0] == '$GPGGA'):
      latitude = splitline[2]
      latDirec = splitline[3]
      longtitude = splitline[4]
      longDirec = splitline[5]
      print("We're at "+latitude+" latitude and "+longtitude+" longtitude")
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