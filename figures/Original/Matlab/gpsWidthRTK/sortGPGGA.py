#!/usr/bin/env python

#allow use of regular expressions
import re

startTime = 0;

readFile  = "data1/solution.NMEA"
writeFile = "sortedGPGGA.csv"

#reading read and write filestreams
with open(readFile, "r") as fileStream:
    with open(writeFile, "w") as fileStreamTwo:
        
        fileStreamTwo.write("# hours, minutes, seconds, seconds elapsed, "
                            "latitude min, latitude deg, "
                            "longitude min, longitude deg, "
                            "fix, altitude \n")
        
        #loops through all the lines in the input file
        for line in fileStream:
            currentLine = line.split(",") #<--make string array from
                                          #   comma separated values
                                          #   in NMEA message
            
            #Evaluating data only in GPGGA strings
            if currentLine[0] == '$GPGGA':
                
                #using regular expressions to separate time-units
                time = re.match('(\d\d)(\d\d)(\d\d)', currentLine[1])
                hours   = time.group(1)
                minutes = time.group(2)
                seconds = time.group(3)
                
                #writing time to csv-file
                fileStreamTwo.write(hours+", "+minutes+", "+seconds+", ")
               
                #----calculating seconds elapsed since start of test---------------
                # 
                hours   = int(hours)
                minutes = int(minutes)
                seconds = int(seconds)
                
                #seconds since last 00:00:00 UTC
                elapSec = seconds + minutes*60 + (hours*60)*60
                
                #start time of test in seconds since last 00:00:00 UTC
                if startTime == 0:
                    startTime = elapSec

                #elapsed seconds since starting time (with potential overflow)
                elapSec -= startTime
                
                #correcting overflow (which happens when time passes 00:00:00 UTC)
                while elapSec < 0:
                    elapSec += (24*60*60)
                
                elapSec = str(elapSec)
                #
                #------------------------------------------------------------------
                
                #writing total elapsed second since test start to csv-file
                fileStreamTwo.write(elapSec+", ")

                #isolating minutes and degrees in latitude and writing to csv
                lat = re.match('(\d\d)(.*)', currentLine[2])
                latMin = lat.group(1)
                latDeg = lat.group(2)
                fileStreamTwo.write(latMin+", "+latDeg+", ")
                
                #isolatng minutes and degrees in longitude and writing to csv
                lng = re.match('(\d\d\d)(.*)', currentLine[4])
                lngMin = lng.group(1)
                lngDeg = lng.group(2)
                fileStreamTwo.write(lngMin+", "+lngDeg+", ")
                
                #writing gps fix to csv-file
                fileStreamTwo.write(currentLine[6]+", ")
                
                #writing attitude (in meter) to csv-file
                fileStreamTwo.write(currentLine[9]+"\n")

