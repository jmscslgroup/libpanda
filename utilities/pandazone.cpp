/*
 Author: Matt Bunting
 Copyright (c) 2020 Arizona Board of Regents
 All rights reserved.

 Permission is hereby granted, without written agreement and without
 license or royalty fees, to use, copy, modify, and distribute this
 software and its documentation for any purpose, provided that the
 above copyright notice and the following two paragraphs appear in
 all copies of this software.

 IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
 FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
 ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
 IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 SUCH DAMAGE.

 THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
 IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
 TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 */

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <cstring>

#include <fstream>
#include <json/json.h>
#include <cmath>



#include "panda.h"

class Vertex;

static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}

// A simple concrete instance of a CAN listener
class SimpleCanObserver : public Panda::CanListener {
public:
	int getNotificationCount() {
		return notificationCount;
	}
private:

	int notificationCount = 0;
	void newDataNotification( Panda::CanFrame* canData ) {
		notificationCount++;
		if(notificationCount > 1000) {
			std::cerr << "c";
			notificationCount = 0;
		}
        
	}

};

// A simple concrete instance of a GPS listener
class SimpleGpsObserver : public Panda::GpsListener {
private:
	int notificationCount = 0;
	void newDataNotification( Panda::GpsData* gpsData ) {
		notificationCount++;
		if(notificationCount > 10) {
			std::cerr << "g";
			notificationCount = 0;
		}
	}
};



// A simple concrete instance of a GPS listener
class SimpleHealthObserver : public Panda::HeartbeatHelperListener {
private:
	int notificationCount = 0;
	void notificationHeartbeat(const PandaHealth& health)  {
		notificationCount++;
		if(notificationCount > 2) {
			std::cerr << "h";
			notificationCount = 0;
		}
	}
};

/*
 Argument setup
 */
void printUsage(const char* binary) {
	std::cout << "Usage: " << binary << " -[v] -[f] [-u usbmode] [-g gpsfile] [-c csvfile] [-n nmeafile]  [-r canfile]" << std::endl;
	std::cout << "   -v          : Verbose mode" << std::endl;
	std::cout << "   -f          : Forces Nissan VIN in configuration for Panda CAN FD" << std::endl;
	std::cout << "   -u usbmode  : USB operating mode:" << std::endl;
	std::cout << "                   a: Asynchronous" << std::endl;
	std::cout << "                   s: Synchronous" << std::endl;
	std::cout << "                   i: Isochronous (not supported)" << std::endl;
	std::cout << "   -g gpsfile  : Filename to output GPS data in CSV format" << std::endl;
//	std::cout << "   -n nmeafile : Filename to output GPS NMEA strings" << std::endl;
	std::cout << "   -c csvfile  : Filename to output CSV format CAN data" << std::endl;
//	std::cout << "   -r canfile  : Filename to save raw messages from Panda CAN reads" << std::endl;
}

int verboseFlag = false;
int forceNissan = false;

static struct option long_options[] =
{
	{"verbose",    no_argument, &verboseFlag, 0},
	{"forcenissan",    no_argument, &forceNissan, 0},
	{"usbmode",    required_argument, NULL, 'u'},
	{"gpsfile",    required_argument, NULL, 'g'},
	{"gpscsvfile",    required_argument, NULL, 'n'},
	{"cancsvfile", required_argument, NULL, 'c'},
	{"canrawfile", required_argument, NULL, 'r'},
	{NULL, 0, NULL, 0}
};

void writeToFileThenClose(const char* filename, const char* data) {
	FILE* file = fopen( filename, "w+");
	fwrite( data, 1, strlen(data), file);
	fclose(file);
};

class Vertex {
public:
    double x;
    double y;
};

class Edge {
private:
    
public:
    Vertex start;
    Vertex end;
    
    virtual ~Edge() = 0;
};

Edge::~Edge() {};

class Straight : public Edge {
public:
    ~Straight() {};
};

class Arc : public Edge {
public:
    double radius;
    Vertex center;
    ~Arc() {};
};

class Polygon {
private:
    std::vector<Edge*> edges;
    
    void makeCircle( const Vertex& center, const double& radius) {
        Arc* circle = new Arc;
        
        // radius is specified in m, but for simple maths let's convert to gps coordinates
        // crudely, 1 degree == 111km
        
        circle->radius = radius * 1.0/111000.0;
        circle->center = center;
        
        circle->start.y = center.y;
        circle->start.x = center.x - circle->radius;
        
        circle->end = circle->start;
        
        edges.push_back(circle);
    }
    
public:
    
    Polygon(Json::Value& region) {
        std::cout << " - Type " << region["type"] << std::endl;
        
        if( region["type"].asString().compare("circle") == 0 ) {
            Vertex center;
            center.x = region["data"]["center"][0].asDouble();
            center.y = region["data"]["center"][1].asDouble();
            makeCircle(center, region["data"]["radius"].asDouble());
        }
        
    }
    
    void print() {
        std::cout << " - This polygon had " << edges.size() << " edges" << std::endl;
        for(std::vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it ++) {
            Edge* edge = *it;
            std::cout << " - Start " << edge->start.x << ", " << edge->start.y << std::endl;
            std::cout << " -   End " << edge->end.x << ", " << edge->end.y << std::endl;
        }
    }
    
    bool isInside(Vertex point) {
        if(edges.size() == 1 &&
           dynamic_cast<Arc*>(edges[0]) != nullptr ) {  // oof
            Arc* zone = dynamic_cast<Arc*>(edges[0]);
            // this is a pure circle;
            
            double distanceToCenter = sqrt(pow(point.x - zone->center.x, 2.0) + pow(point.y - zone->center.y, 2.0));
            
            if(distanceToCenter > zone->radius) {
                return false;
            }
        }
        return true;
    }
    
    ~Polygon() {
        for(std::vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
            delete *it;
        }
    }
};

class ZoneChecker {
private:
    // Json parsing
    std::vector<Polygon*> polygons;
    
public:
    void open( const char* file) {
        std::ifstream zoneFile;
        zoneFile.open(file);
        if( errno != 0) {
            std::cerr << "Json Error: " << std::strerror(errno) << std::endl;
        } else {
            Json::Value zoneDefinition;
            zoneFile >> zoneDefinition;
            zoneFile.close();
            
            Json::StreamWriterBuilder builder;
            const std::string output = Json::writeString(builder, zoneDefinition);
            std::cout << output << std::endl;
            std::cout << "Zonefile created " << zoneDefinition["created_at"] << std::endl;
            std::cout << "Zonefile associated with VIN " << zoneDefinition["vin"] << std::endl;
            std::cout << "Zonefile has " << zoneDefinition["regions"].size() << " regions" << std::endl;
            
            for(int i = 0; i < zoneDefinition["regions"].size(); i++) {
                std::cout << " - Region " << i << std::endl;
                polygons.push_back(new Polygon(zoneDefinition["regions"][i]));
            }
        }
        
        std::cout << "Parsed polygons:" << std::endl;
        for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
            Polygon *polygon = *it;
            polygon->print();
        }
        
//        std::cout << "Check if point is inside:" << std::endl;
//        Vertex point;
//        point.x = 100;
//        point.y = -3;
//        Vertex point2;
//        point2.x = -86.62;
//        point2.y = 36.029;
//        for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
//            Polygon* polygon = *it;
//            std::cout << " - - IsInside point1: " << polygon->isInside(point) << std::endl;
//            std::cout << " - - IsInside point2: " << polygon->isInside(point2) << std::endl;
//        }
        
        
    }
    
    ~ ZoneChecker() {
        // Cleanup
        for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
            delete *it;
        }
    }
    
    bool inZone(Vertex point) {
        bool result = false;
        
        for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
            Polygon* polygon = *it;
            if(polygon->isInside(point)) {
                return true;
            }
        }
        
        return result;
    }
};

class CsvRecorder : public Panda::GpsListener, public Panda::CanListener {
private:
    std::string gpsFilename;
    std::string canFilename;
    
    char timeStamp[200];
    
    ZoneChecker* zoneChecker;
    
    FILE* csvGpsDump;
    FILE* csvCanDump;
    bool busyFileGps;
    bool busyFileCan;
    
    bool recordingEnabled;
    bool recordingAllowed;
    
    void openCanFile() {
        busyFileWaitCan();
        busyFileCan = true;
        std::stringstream newFileName;
        newFileName << canFilename;
        newFileName << timeStamp;
        FILE* csvDumpTemp = fopen(newFileName.str().c_str(), "w");
        fprintf(csvDumpTemp, "Time,Bus,MessageID,Message,MessageLength\n");
        this->csvCanDump = csvDumpTemp;
        remove("/etc/libpanda.d/currentCan.csv");
        symlink(newFileName.str().c_str(),"/etc/libpanda.d/currentCan.csv");
        busyFileCan = false;
    }
    
    void openGpsFile() {
        busyFileWaitGps();
        busyFileGps = true;
        std::stringstream newFileName;
        newFileName << gpsFilename;
        newFileName << timeStamp;
        FILE* csvDumpTemp = fopen(newFileName.str().c_str(), "w");
        fprintf(csvDumpTemp, "Gpstime,Status,Long,Lat,Alt,HDOP,PDOP,VDOP,Systime\n");
        this->csvGpsDump = csvDumpTemp;
        remove("/etc/libpanda.d/currentGps.csv");
        symlink(newFileName.str().c_str(),"/etc/libpanda.d/currentGps.csv");
        busyFileGps = false;
    }
    
    void closeCanFile() {
        busyFileWaitCan();
        busyFileCan = true;
        fclose(csvCanDump);
        csvCanDump = NULL;
        busyFileCan = false;
    }
    
    void closeGpsFile() {
        busyFileWaitGps();
        busyFileGps = true;
        fclose(csvGpsDump);
        csvGpsDump = NULL;
        busyFileGps = false;
    }
    
    
    void busyFileWaitCan() {
        while(busyFileCan) {
            usleep(100);
        }
    }
    void busyFileWaitGps() {
        while(busyFileGps) {
            usleep(1000);
        }
    }
    
    
    
    void newDataNotification( Panda::GpsData* gpsData ) {
    
        if(zoneChecker) {
            // check if data still allowed:
            Vertex pose;
            
            pose.x = gpsData->pose.longitude;
            pose.y = gpsData->pose.latitude;
            
            if( !zoneChecker->inZone(pose) ) {
                
                if(recordingAllowed) {
                    std::cout << "Not in zone!  Not recording" << std::endl;
                    closeCanFile();
                    closeGpsFile();
                }
                recordingAllowed = false;
            } else {
                if(!recordingAllowed) {
                    std::cout << "inside zone!  safe to record" << std::endl;

                    if(recordingEnabled) {
//                        struct timeval sysTime;
//                        gettimeofday(&sysTime, NULL);
//                        sprintf(timeStamp, "%ld.%06d.csv", (int)sysTime.tv_sec,  (int)sysTime.tv_usec);
                        
                        openFiles();
                    }
                }
                recordingAllowed = true;
            }
        }
        
        if(!(recordingEnabled && recordingAllowed)) {
            return;
        }
        
        busyFileWaitGps();
        busyFileGps = true;
        if (csvGpsDump) {
            time_t gpsTime_t = mktime(&gpsData->time);
    //        struct timeval gpsTime;
    //        gpsTime.tv_sec = gpsTime_t;
    //        gpsTime.tv_usec = (state.timeMilliseconds)*1000;
            
            // Get system time to check against GPS time skew
            struct timeval sysTime;
            gettimeofday(&sysTime, NULL);

            fprintf(csvGpsDump, "%d.%06d,%c,%0.7f,%0.7f,%0.1f,%0.2f,%0.2f,%0.2f,%d.%06d\r\n",
                    (unsigned int)gpsTime_t,
                    (gpsData->timeMilliseconds)*1000,
                    gpsData->info.status,
                    gpsData->pose.longitude,
                    gpsData->pose.latitude,
                    gpsData->pose.altitude,
                    gpsData->quality.HDOP,
                    gpsData->quality.PDOP,
                    gpsData->quality.VDOP,
                    (unsigned int)sysTime.tv_sec,
                    (int)sysTime.tv_usec);
        }
        busyFileGps = false;
    }
    
    
    void newDataNotification( Panda::CanFrame* frame ) {
        if(!(recordingEnabled && recordingAllowed)) {
            return;
        }
        
        busyFileWaitCan();
        busyFileCan = true;
        if (csvCanDump != NULL) {
            fprintf(csvCanDump, "%d.%06d,", (unsigned int)frame->sysTime.tv_sec, (int)frame->sysTime.tv_usec);
            //        for (int i = 0; i < bufLength; i++) {
            //            fprintf(csvDump, "%02x", converted[i]);
            //        }
            
            fprintf(csvCanDump,"%d,%u,", (int)frame->bus, frame->messageID);
            
            for (int i =0; i < frame->dataLength; i++) {
                fprintf(csvCanDump, "%02x", frame->data[i]);
            }
            fprintf(csvCanDump, ",%d\n", frame->dataLength);
            
        }
        busyFileCan = false;
    }
    
    void openFiles() {
        time_t t = time(NULL);
        struct tm tm = *localtime(&t);
        
        sprintf(timeStamp, "-%04d-%02d-%02d-%02d-%02d-%02d.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour + tm.tm_isdst, tm.tm_min, tm.tm_sec);
        
        std::cout << "New timestamp appended: " << timeStamp << std::endl;
        
        openGpsFile();
        openCanFile();
    }
    
public:
    CsvRecorder( ) {
        busyFileGps = false;
        busyFileCan = false;
        
        recordingAllowed = false;
        recordingEnabled = false;
        
        zoneChecker = NULL;
    }
    
    void setZoneChecker(ZoneChecker* checker) {
        zoneChecker = checker;
    }
    
    void saveToCsvFiles(const char* gps, const char* can) {
        gpsFilename = gps;
        canFilename = can;
        
        recordingEnabled = true;
        if(recordingAllowed) {
            openFiles();
        }
    }
    
    void closeFiles() {
        closeCanFile();
        closeGpsFile();
    }
};

using namespace std;
int main(int argc, char **argv) {
    
    ZoneChecker zCheck;
    zCheck.open("/etc/libpanda.d/zone-testbed.json");
    
    CsvRecorder mCsvRecorder;
    mCsvRecorder.setZoneChecker(&zCheck);
//    // Json parsing
//    std::vector<Polygon*> polygons;
//
//    std::ifstream zoneFile;
//    zoneFile.open("/etc/libpanda.d/zone-testbed.json");
//    if( errno != 0) {
//        std::cerr << "Errror: " << std::strerror(errno) << std::endl;
//    } else {
//        Json::Value zoneDefinition;
//        zoneFile >> zoneDefinition;
//        zoneFile.close();
//
//        Json::StreamWriterBuilder builder;
//        const std::string output = Json::writeString(builder, zoneDefinition);
//        std::cout << output << std::endl;
//        std::cout << "Zonefile created " << zoneDefinition["created_at"] << std::endl;
//        std::cout << "Zonefile associated with VIN " << zoneDefinition["vin"] << std::endl;
//        std::cout << "Zonefile has " << zoneDefinition["regions"].size() << " regions" << std::endl;
//
//        for(int i = 0; i < zoneDefinition["regions"].size(); i++) {
//            std::cout << " - Region " << i << std::endl;
//            polygons.push_back(new Polygon(zoneDefinition["regions"][i]));
//        }
//    }
//
//    std::cout << "Parsed polygons:" << std::endl;
//    for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
//        Polygon *polygon = *it;
//        polygon->print();
//    }
//
//    std::cout << "Check if point is inside:" << std::endl;
//    Vertex point;
//    point.x = 100;
//    point.y = -3;
//    Vertex point2;
//    point2.x = -86.62;
//    point2.y = 36.029;
//    for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
//        Polygon* polygon = *it;
//        std::cout << " - - IsInside point1: " << polygon->isInside(point) << std::endl;
//        std::cout << " - - IsInside point2: " << polygon->isInside(point2) << std::endl;
//    }
//
//    // Cleanup
//    for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
//        delete *it;
//    }
    
	// Argument parsing
	Panda::UsbMode usbMode = Panda::MODE_ASYNCHRONOUS;
	const char*    gpsFilename = NULL;
	const char*   nmeaFilename = NULL;
	const char* canCsvFilename = NULL;
	const char* canRawFilename = NULL;
	int ch;
	while ((ch = getopt_long(argc, argv, "u:g:c:vf", long_options, NULL)) != -1)
	{
		switch (ch)
		{
			case 'u':
				switch (optarg[0]) {
					case 'a': usbMode = Panda::MODE_ASYNCHRONOUS; break;
					case 's': usbMode =  Panda::MODE_SYNCHRONOUS; break;
					case 'i': usbMode =  Panda::MODE_ISOCHRONOUS; break; };
				break;
			case 'g':    gpsFilename = optarg; break;
//			case 'n':   nmeaFilename = optarg; break;
			case 'c': canCsvFilename = optarg; break;
//			case 'r': canRawFilename = optarg; break;
			case 'f':  forceNissan = true; break;
			default:
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
				break;
		}
	}

	const char filenamePandaStatus[] = "/etc/libpanda.d/pandaRecording";
	const char filenameGpsStatus[] = "/etc/libpanda.d/pandaHaveGPS";
	writeToFileThenClose(filenamePandaStatus, "-1\n");
	writeToFileThenClose(filenameGpsStatus, "-1\n");

	std::cout << "Starting " << argv[0] << std::endl;

	//Set up graceful exit
	signal(SIGINT, killPanda);

	SimpleCanObserver canObserver;
	SimpleGpsObserver myGpsObserver;
	SimpleHealthObserver mySimpleHealthObserver;

	double epsilon = 0.2;	// If system time is off from GPS time by this amount, update time.
	Panda::SetSystemTimeObserver mSetSystemTimeObserver(epsilon);
	
	Panda::GpsTracker mGpsTracker;	// Saves to /etc/libpanda.d/latest_gps

	// Initialize Usb, this requires a conencted Panda
	Panda::Handler pandaHandler;
	pandaHandler.getUsb().setOperatingMode(usbMode);
	pandaHandler.addCanObserver(canObserver);
	pandaHandler.addGpsObserver(myGpsObserver);
    pandaHandler.addCanObserver(mCsvRecorder);
    pandaHandler.addGpsObserver(mCsvRecorder);
//	pandaHandler.addGpsObserver(mSetSystemTimeObserver);
	pandaHandler.addGpsObserver(mGpsTracker);
	pandaHandler.addHeartbeatObserver(mySimpleHealthObserver);

	// Let's roll
	if (forceNissan) {
		pandaHandler.initialize((const unsigned char*)"JN8AT3CB9MW240939");
	} else {
		pandaHandler.initialize();
	}

	writeToFileThenClose(filenameGpsStatus, "0\n");	// state 0: on but time not set

	int lastNmeaMessageCount = 0;
//	if (pandaHandler.getUsb().hasGpsSupport()) {
	if (pandaHandler.getGps().available()) {
		pandaHandler.addGpsObserver(mSetSystemTimeObserver);
		std::cout << "Waiting to acquire satellites to set system time..." << std::endl;
		std::cout << " - Each \'.\' represents 100 NMEA messages received:" << std::endl;
		
		while ( !mSetSystemTimeObserver.hasTimeBeenSet() &&
			   keepRunning == true ) {
			if (pandaHandler.getGps().getData().successfulParseCount-lastNmeaMessageCount > 100) {
				std::cerr << ".";
				lastNmeaMessageCount = pandaHandler.getGps().getData().successfulParseCount;
			}
			usleep(10000);
		}
		writeToFileThenClose(filenameGpsStatus, "1\n");	// GPS time sync done
		writeToFileThenClose(filenamePandaStatus, "0\n"); // Recording should now start
	} else {
		std::cout << "This system does not have a GPS, continuing to log data without time synchronization" << std::endl;
		writeToFileThenClose(filenamePandaStatus, "0\n"); // Recording should now start
	}


	if (gpsFilename != NULL && canCsvFilename != NULL) {
//		pandaHandler.getGps().saveToCsvFile(gpsFilename);
//        myGpsObserver.saveToCsvFile(gpsFilename);
        mCsvRecorder.saveToCsvFiles(gpsFilename, canCsvFilename);
	}
//	if (nmeaFilename != NULL) {
//		pandaHandler.getGps().saveToFile(nmeaFilename);
//	}
//	if (canCsvFilename != NULL) {
//		pandaHandler.getCan().saveToCsvFile(canCsvFilename);
//	}
//	if (canRawFilename != NULL) {
//		pandaHandler.getCan().saveToFile(canRawFilename);
//	}
	

	std::cout << "Time is synced with GPS!" << std::endl;
	std::cout << std::endl << "Press ctrl-c to exit" << std::endl;
	std::cout << " - Each \'c\' represents 1000 CAN notifications received." << std::endl;
	std::cout << " - Each \'.\' represents 100 NMEA messages received." << std::endl;
	std::cout << " - Each \'g\' represents 10 GPS notifications received." << std::endl;
	std::cout << " - Each \'h\' represents 2 Heartbeat/Health notifications received." << std::endl;

	int priorCanCount = 0;
	int gpsHeartbeat = 0;
	while (keepRunning == true) {
		// GPS status checking:
		pandaHandler.getUsb().sendHeartBeat();
		if (pandaHandler.getGps().getData().successfulParseCount-lastNmeaMessageCount > 100) {
			std::cerr << ".";
			lastNmeaMessageCount = pandaHandler.getGps().getData().successfulParseCount;
			gpsHeartbeat = 0;
			writeToFileThenClose(filenameGpsStatus, "1\n");
		}
		gpsHeartbeat++;
		if(gpsHeartbeat > 100) {
			writeToFileThenClose(filenameGpsStatus, "0\n");
		}

		// CAN recording status checking:
		if (priorCanCount == canObserver.getNotificationCount()) {
			writeToFileThenClose(filenamePandaStatus, "0\n");
			priorCanCount = canObserver.getNotificationCount();
		} else {
			writeToFileThenClose(filenamePandaStatus, "1\n");
		}



		usleep(10000);
	}
	writeToFileThenClose(filenamePandaStatus, "0\n");
	writeToFileThenClose(filenameGpsStatus, "0\n");
	//pandaHandler.stop();
	pandaHandler.stop();

	std::cout << "\rDone." << std::endl;
	return 0;
}
