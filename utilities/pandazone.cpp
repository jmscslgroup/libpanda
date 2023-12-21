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

#include <sstream>

#include "panda.h"

class Vertex;

static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}

void printTenThenNewLine(const char* s)  {
    static int characterCount = 0;
    if(characterCount++ > 100) { // Oops it's not 10 anymore *shrugs*
        characterCount = 0;
        std::cerr << s << std::endl;
    } else {
        std::cerr << s;
    }
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
//			std::cerr << "c";
            printTenThenNewLine("c");
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
//			std::cerr << "g";
            printTenThenNewLine("g");
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
//			std::cerr << "h";
            printTenThenNewLine("h");
			notificationCount = 0;
		}
	}
};

/*
 Argument setup
 */
void printUsage(const char* binary) {
	std::cout << "Usage: " << binary << " -[v] -[f] [-u usbmode] [-d directory]" << std::endl;
	std::cout << "   -v          : Verbose mode" << std::endl;
	std::cout << "   -f          : Forces Nissan VIN in configuration for Panda CAN FD" << std::endl;
	std::cout << "   -u usbmode  : USB operating mode:" << std::endl;
	std::cout << "                   a: Asynchronous" << std::endl;
	std::cout << "                   s: Synchronous" << std::endl;
	std::cout << "                   i: Isochronous (not supported)" << std::endl;
//	std::cout << "   -g gpsfile  : Filename to output GPS data in CSV format" << std::endl;
//	std::cout << "   -n nmeafile : Filename to output GPS NMEA strings" << std::endl;
//	std::cout << "   -c csvfile  : Filename to output CSV format CAN data" << std::endl;
    std::cout << "   -d directory: Directory for gps/can data dump" << std::endl;
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
    
    Vertex() :
    x(0), y(0) {};
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
protected:
    std::vector<Edge*> edges;
    std::vector<double> vertexAngles;
    std::vector<Straight> normals;
    bool rightHanded;
    
    Polygon* hysteresisBoundary;
   // std::vector<Edge*> offsetEdges;
    
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
    
    double square(double x) {
        return x*x;
    }
    double magnitude(Edge& edge) {
        return sqrt(square(edge.start.x - edge.end.x) + square(edge.start.y - edge.end.y));
    }
    double magnitude(Vertex& vertex) {
        return sqrt(square(vertex.x) + square(vertex.y));
    }
    
    void determingVertexAngles() {
        Edge* priorEdge = *(edges.end()-1);
        
        for(std::vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
            Edge* currentEdge = *it;
            
            Straight priorVector;
            priorVector.end.x = priorEdge->end.x - priorEdge->start.x;
            priorVector.end.y = priorEdge->end.y - priorEdge->start.y;
            
            Straight currentVector;
            currentVector.end.x = currentEdge->end.x - currentEdge->start.x;
            currentVector.end.y = currentEdge->end.y - currentEdge->start.y;
            
            // cross product for a-plane directiondirection
            double z = priorVector.end.x * currentVector.end.y - priorVector.end.y * currentVector.end.x;
            if(rightHanded) {
                z = -z;
            }
            // dot product for use of arccos's domain
            double dot = priorVector.end.x * currentVector.end.x + priorVector.end.y * currentVector.end.y;
            double angle = acos( dot / (magnitude(priorVector) * magnitude(currentVector)));
            if(z > 0) {
//                angle += M_PI;
//                angle = 2*M_PI - angle;
                angle = M_PI + angle;
            } else {
                angle =  M_PI - angle;
            }
            
            vertexAngles.push_back( angle );
            
            priorEdge = currentEdge;
        }
        
        double sumAngles = 0;
        for(int i = 0; i < vertexAngles.size(); i++) {
            sumAngles += vertexAngles[i];
        }
//        std::cout << "Sumangle:" << sumAngles*(180.0/M_PI) << " size: " << vertexAngles.size() << std::endl;
        if(fabs(sumAngles/M_PI - (double)(vertexAngles.size()-2)) > 0.5  &&
           rightHanded) {
//            std::cout << "Sumangle too far off!  Changing handedness" << std::endl;
            rightHanded = false;
            vertexAngles.clear();
            determingVertexAngles();
        }
        
        
    }

    void computeNormals() {
        Straight normal;
        
        for(std::vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
            Edge* edge = *it;
            double mag = magnitude(*edge);
            normal.end.x = (edge->end.y - edge->start.y)/mag;
            normal.end.y = -(edge->end.x - edge->start.x)/mag;
            
            if(!rightHanded) {
                normal.end.x = -normal.end.x;
                normal.end.y = -normal.end.y;
            }
            
            
//            std::cout << "Normal: " << normal.end.x << ", " << normal.end.y << std::endl;
//            std::cout << " - mag: " << mag << std::endl;
            
            normals.push_back(normal);
        }
    }
    
    // for compute hysteresis
    Polygon() {
        
    }
    
public:
    
    Polygon(Json::Value& region) {
        rightHanded = true;
        hysteresisBoundary = NULL;
        std::cout << " - Type " << region["type"] << std::endl;
        
        if( region["type"].asString().compare("circle") == 0 ) {
            Vertex center;
            center.x = region["data"]["center"][0].asDouble();
            center.y = region["data"]["center"][1].asDouble();
            makeCircle(center, region["data"]["radius"].asDouble());
        } else if(region["type"].asString().compare("polygon") == 0 ){
            if(region["data"].size() < 3) {
                std::cerr << "Warning!  Polygon under defined with " << region["data"].size() << " vertices" << std::endl;
                return;
            }
            std::cout << "This polygon has " << region["data"].size() << " elements, so total vertices: " << region["data"].size()-1 << std::endl;
            Straight* straight = new Straight;
            straight->start.x = region["data"][region["data"].size()-2][0].asDouble();
            straight->start.y = region["data"][region["data"].size()-2][1].asDouble();
//            straight->start.x = region["data"][0][0].asDouble();
//            straight->start.y = region["data"][0][1].asDouble();
            
            int i;
            for(i = 0; i < region["data"].size()-1; i++) {
//                std::cout << "straight->start.x = " << straight->start.x;
                straight->end.x = region["data"][i][0].asDouble();
                straight->end.y = region["data"][i][1].asDouble();
                
//                    std::cout << ", end.x = " << straight->end.x << std::endl;
                
                edges.push_back(straight);
                
                Straight* prior = straight;
                straight = new Straight;
                straight->start = prior->end;
            }
            delete straight;
            
//            std::cout << "Resulting edge count: " << edges.size() << std::endl;
//            straight->end.x = region["data"][i][0].asDouble();
//            straight->end.y = region["data"][i][1].asDouble();
//
//            std::cout << ", end.x = " << straight->end.x << std::endl;
//            edges.push_back(straight);
            
            determingVertexAngles();
            
            computeNormals();
        }
        
    }
    
    void computeOffsetPolygon(double offsetMeters) {
        hysteresisBoundary = new Polygon;
        if(edges.size() == 1) { // is a circle?
            Arc* circle = new Arc;
            
            *circle = *(Arc*)(edges[0]);
            
            circle->radius += offsetMeters/111000.0;
            
            hysteresisBoundary->edges.push_back(circle);
        } else if ( edges.size() >= 3 ) {
//            Edge* priorEdge = *(edges.end()-1);
//            Straight priorNormal = *(normals.end()-1);
            Edge* priorEdge = edges[edges.size()-1];
            Straight priorNormal = normals[normals.size()-1];
            
            double offsetDegrees = offsetMeters/111000.0;
//            std::cout << "Offset of " << offsetDegrees << std::endl;
            for(int i = 0; i < edges.size(); i++) {
                Edge* currentEdge = edges[i];
                Straight currentNormal = normals[i];
                
                
                Straight* offsetStraight = new Straight;
                
                Vertex sumNormals;
                sumNormals.x = priorNormal.end.x + currentNormal.end.x;
                sumNormals.y = priorNormal.end.y + currentNormal.end.y;
                
//                std::cout << "magnitude(sumNormals) = " << magnitude(sumNormals) << std::endl;
                
                Vertex sumNormalsNormal;
                sumNormalsNormal.x = sumNormals.x/magnitude(sumNormals);
                sumNormalsNormal.y = sumNormals.y/magnitude(sumNormals);
                
                double halfDist = magnitude(sumNormals)/2;
                double d = halfDist + (1.0 - square(halfDist))/halfDist;
                
                offsetStraight->start.x = d * offsetDegrees * sumNormalsNormal.x + currentEdge->start.x;
                offsetStraight->start.y = d * offsetDegrees * sumNormalsNormal.y + currentEdge->start.y;
                
                hysteresisBoundary->edges.push_back(offsetStraight);
                priorEdge = currentEdge;
                priorNormal = currentNormal;
            }
            
            // All start vertices filled, now copy current start into prior end to complete:
//            priorEdge = *(hysteresisBoundary->edges.end()-1);
            priorEdge = hysteresisBoundary->edges[hysteresisBoundary->edges.size() - 1];
            for(std::vector<Edge*>::iterator it = hysteresisBoundary->edges.begin(); it != hysteresisBoundary->edges.end(); it++) {
                Edge* currentEdge = *it;
                
                priorEdge->end = currentEdge->start;
                
                priorEdge = currentEdge;
//                std::cout << " - -priorEdge->end.x = " << priorEdge->end.x << std::endl;
            }
            
            
        }
        
        std::cout << " - H size: " << hysteresisBoundary->edges.size() << std::endl;
    }
    
    void print() {
        double angleSum = 0;
        std::cout << " - This polygon had " << edges.size() << " edges" << std::endl;
//        for(std::vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it ++) {
        for(int i = 0; i < edges.size(); i++) {
            Edge* edge = edges[i];
            std::cout << " - -  Start " << edge->start.x << ", " << edge->start.y << std::endl;
            std::cout << " - -  End " << edge->end.x << ", " << edge->end.y << std::endl;
            
            
            if( i < vertexAngles.size()) {
                double angle = vertexAngles[i];
                std::cout << " - - Angle " << angle << std::endl;
                angleSum += vertexAngles[i];
                std::cout << " - - - Normal " << normals[i].end.x << ", " << normals[i].end.y << std::endl;
              }
        
        }
        
        if(vertexAngles.size() > 0) {
            std::cout << " - Angle sum in degrees: " << (180.0/3.1415926535897 * angleSum) << std::endl;
        }
        
        if(hysteresisBoundary) {
            
            std::cout << "!!!!! the follwoing is the hysteresis bound for the above pollygon:" << std::endl;
            hysteresisBoundary->print();
        }
    }
    
    bool isInsideHysteresisBound(Vertex point) {
        if(hysteresisBoundary) {
            return hysteresisBoundary->isInside(point);
        }
        std::cout << "No hysteresis bound defined!  Checking against base zone instead" << std::endl;
        return isInside( point);
    }
    
    bool isInside(Vertex point) {
        if(edges.size() == 1 &&
           dynamic_cast<Arc*>(edges[0]) != nullptr ) {  // oof
            Arc* zone = dynamic_cast<Arc*>(edges[0]);
            // this is a pure circle;
            
            double distanceToCenter = sqrt(pow(point.x - zone->center.x, 2.0) + pow(point.y - zone->center.y, 2.0));
            
            
//                std::cout << "Circle check" << std::endl;
            if(distanceToCenter > zone->radius) {
//                std::cout << " - This is not inside circle!" << std::endl;
                return false;
            }
        } else {
            // polygon
            int intersectCount = 0;
            int i = 0;
            for(std::vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++, i++) {
                Edge* edge = *it;
                // Think of casting a ray horizontally, from left to right
                // if both y are larger than the point, or both less, then it does not intersect
                // Same if both are larger than x
                if(edge->start.y > point.y &&
                   edge->end.y > point.y) {
                    continue;
                } else if(edge->start.y < point.y &&
                          edge->end.y < point.y) {
                    continue;
                } else if (edge->start.x > point.x &&
                           edge->end.x > point.x) {
                    continue;
                }
                
                // Find intersection point, and see if intercetion point's x is larger than the point
                double intersectX = (edge->start.x - edge->end.x)/(edge->start.y - edge->end.y) * (point.y - edge->end.y) + edge->end.x;
                
//                std::cout << "i: " << i << " interxectX: " << intersectX << "  point.x: " << point.x << std::endl;
                if(intersectX <= point.x) {
                    intersectCount++;
                }
                
            }
//            std::cout << "Intersect count result: " << intersectCount << " For edge count " << edges.size() << std::endl;
            if(intersectCount % 2 == 0) {
//                std::cout << " - This is not inside!" << std::endl;
                return false;
            }
        }
        
        
        return true;
    }
    
    ~Polygon() {
        for(std::vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
            delete *it;
        }
        
//        for(std::vector<Edge*>::iterator it = offsetEdges.begin(); it != offsetEdges.end(); it++) {
//            delete *it;
//        }
        if(hysteresisBoundary) {
            delete hysteresisBoundary;
        }
    }
    
    Json::Value toJson() {
        Json::Value region;
        if(edges.size() == 1) {
            std::cout << " - formatting Circle" << std::endl;
            region["type"] = "circle";
            region["data"]["radius"] = ((Arc*)edges[0])->radius * 111000.0;
            region["data"]["center"][0] = ((Arc*)edges[0])->center.x;
            region["data"]["center"][1] = ((Arc*)edges[0])->center.y;
        } else {
            std::cout << " - formatting polygon" << std::endl;
            region["type"] = "polygon";
            for(int i = 0; i < edges.size(); i++) {
                region["data"][i][0] = edges[i]->start.x;
                region["data"][i][1] = edges[i]->start.y;
            }
            region["data"][(int)edges.size()][0] = edges[0]->start.x;    // needs to be connected
            region["data"][(int)edges.size()][1] = edges[0]->start.y;
        }
        if(hysteresisBoundary) {
            region["hysteresis"] = hysteresisBoundary->toJson();
        }
        return region;
    }
};

class ZoneChecker {
private:
    // Json parsing
    std::vector<Polygon*> polygons;
    std::string vin;
    double offset;
public:
    void save(const char* filename) {
        Json::Value zoneDefinition;
        zoneDefinition["vin"] = vin;
        zoneDefinition["offset"] = offset;
        
        for(int i = 0; i < polygons.size(); i++) {
            std::cout << "In polygon #" << i << std::endl;
            zoneDefinition["regions"][i] = polygons[i]->toJson();
        }
        
        std::ofstream file_id;
        file_id.open(filename);
        if( file_id.fail() ) {
            std::cerr << "Failed to write to " << filename << " permissions issue?" << std::endl;
            return;
        }
        
        Json::StyledWriter styledWriter;
        file_id << styledWriter.write(zoneDefinition);
        
        file_id.close();
        
    }
    
    int open( const char* file) {
        std::ifstream zoneFile;
        zoneFile.open(file);
        if( zoneFile.fail() ) {
            std::cerr << "Json Error: " << std::strerror(errno) << std::endl;
            return 1;
        } else {
            Json::Value zoneDefinition;
            zoneFile >> zoneDefinition;
            zoneFile.close();
            
            Json::StreamWriterBuilder builder;
            const std::string output = Json::writeString(builder, zoneDefinition);
            vin = zoneDefinition["vin"].asString();
//            std::cout << output << std::endl;
//            std::cout << "Zonefile created " << zoneDefinition["created_at"] << std::endl;
//            std::cout << "Zonefile associated with VIN " << zoneDefinition["vin"] << std::endl;
//            std::cout << "Zonefile has " << zoneDefinition["regions"].size() << " regions" << std::endl;
            
            for(int i = 0; i < zoneDefinition["regions"].size(); i++) {
//                std::cout << " - Region " << i << std::endl;
                Polygon* region = new Polygon(zoneDefinition["regions"][i]);
                if(zoneDefinition.isMember("offset")) {
                    offset = zoneDefinition["offset"].asDouble();
                } else {
    //                region->computeOffsetPolygon(-200);  // eight mile inward
    //                region->computeOffsetPolygon(111000.0);  // 1 degree
                    offset = -457.2;  // eight mile inward
                }
                region->computeOffsetPolygon(offset);
                polygons.push_back(region);
            }
            
            std::cout << "Loaded zonefile " << file << " with zone count " << polygons.size() << std::endl;
        }
        
//        std::cout << "Parsed polygons:" << std::endl;
//        for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
//            Polygon *polygon = *it;
//            polygon->print();
//        }
        
//        std::cout << "Check if point is inside:" << std::endl;
//        Vertex point;
//        point.x = -86.713167706175582;
//        point.y = 36.107;
//        Vertex point2;
//        point2.x = -86.792709236746688;
//        point2.y = 36.145815819708503;
//        Vertex point3;
//        point3.x = 0.5;
//        point3.y = 1.5;
//        for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
//            Polygon* polygon = *it;
//            std::cout << " - - IsInside point1: " << polygon->isInside(point) << std::endl;
//            std::cout << " - - IsInside point2: " << polygon->isInside(point2) << std::endl;
//            std::cout << " - - IsInside point3: " << polygon->isInside(point3) << std::endl;
////            std::cout << " - - Hyeteresiss point3: " << polygon->isInsideHysteresisBound(point3) << std::endl;
//        }
        
        return 0;
    }
    
    ~ZoneChecker() {
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
    
    bool inHysteresisZone(Vertex point) {
        bool result = false;
        
        for(std::vector<Polygon*>::iterator it = polygons.begin(); it != polygons.end(); it++) {
            Polygon* polygon = *it;
            if(polygon->isInsideHysteresisBound(point)) {
                return true;
            }
        }
        
        return result;
    }
    
    bool zoneIsInclusive() {
        return offset < 0;
    }
};

class CsvRecorder : public Panda::GpsListener, public Panda::CanListener {
private:
    typedef enum {
        STATE_RECORDING,
        STATE_IDLE
    } RecordingState;
    RecordingState state;
    
    std::string gpsFilename;
    std::string canFilename;
    std::string dataDirectory;
    bool beepNotification;
    
    Panda::MatthatBeep beeper;
    
    char timeStamp[200];
    
    ZoneChecker* zoneChecker;
    
    FILE* csvGpsDump;
    FILE* csvCanDump;
    bool busyFileGps;
    bool busyFileCan;
    
    bool recordingEnabled;
//    bool recordingAllowed;
    
    void openCanFile() {
        busyFileWaitCan();
        busyFileCan = true;
//        std::stringstream newFileName;
//        newFileName << canFilename;
//        newFileName << timeStamp;
        FILE* csvDumpTemp = fopen(canFilename.c_str(), "w");
        fprintf(csvDumpTemp, "Time,Bus,MessageID,Message,MessageLength\n");
        this->csvCanDump = csvDumpTemp;
        remove("/etc/libpanda.d/currentCan.csv");
        symlink(canFilename.c_str(),"/etc/libpanda.d/currentCan.csv");
        busyFileCan = false;
    }
    
    void openGpsFile() {
        busyFileWaitGps();
        busyFileGps = true;
//        std::stringstream newFileName;
//        newFileName << gpsFilename;
//        newFileName << timeStamp;
        FILE* csvDumpTemp = fopen(gpsFilename.c_str(), "w");
        fprintf(csvDumpTemp, "Gpstime,Status,Long,Lat,Alt,HDOP,PDOP,VDOP,Systime\n");
        this->csvGpsDump = csvDumpTemp;
        remove("/etc/libpanda.d/currentGps.csv");
        symlink(gpsFilename.c_str(),"/etc/libpanda.d/currentGps.csv");
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
        if( !recordingEnabled ) {    // TODO this can never be set false by user
            return;
        }
        if(zoneChecker) {
            // check if data still allowed:
            Vertex pose;
            
            pose.x = gpsData->pose.longitude;
            pose.y = gpsData->pose.latitude;
            
            switch (state) {
                case STATE_IDLE:
                    if( zoneChecker->zoneIsInclusive() ) {
                        if( zoneChecker->inHysteresisZone(pose) ) {
                            transitionToState(STATE_RECORDING); // Inclusive zone and we have entered the boundary
                        }
                    } else {
                        if( ! zoneChecker->inHysteresisZone(pose) ) {
                            transitionToState(STATE_RECORDING); // Exclusive zone and we have left the boundary
                        }
                    }
                    break;
                    
                    
                case STATE_RECORDING:
                    if( zoneChecker->zoneIsInclusive() ) {
                        if( ! zoneChecker->inZone(pose)) {
                            transitionToState(STATE_IDLE);  // Inclusive zone and we have left the boundary
                        }
                    } else {
                        if( zoneChecker->inZone(pose)) {
                            transitionToState(STATE_IDLE);  // Exclusive zone and we have entered the boundary
                        }
                    }
                    break;
            }
            
//            if( !zoneChecker->inZone(pose) ) {
//                if(recordingAllowed) {
//                    std::cout << "Not in zone!  Not recording" << std::endl;
//                    closeCanFile();
//                    closeGpsFile();
//                }
//                recordingAllowed = false;
//            } else {
//                if(!recordingAllowed) {
//                    std::cout << "inside zone!  safe to record" << std::endl;
//
//                    if(recordingEnabled) {
////                        struct timeval sysTime;
////                        gettimeofday(&sysTime, NULL);
////                        sprintf(timeStamp, "%ld.%06d.csv", (int)sysTime.tv_sec,  (int)sysTime.tv_usec);
//
//                        openFiles();
//                    }
//
//                }
//                recordingAllowed = true;
//            }
        }
        
        if(state != STATE_RECORDING) {
            return;
        }
        
//        if(!(recordingEnabled && recordingAllowed)) {
//            return;
//        }
        
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
//        if(!(recordingEnabled && recordingAllowed)) {
//            return;
//        }
        if(state != STATE_RECORDING) {
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
            
            for (int i = 0; i < frame->dataLength; i++) {
                fprintf(csvCanDump, "%02x", frame->data[i]);
            }
            fprintf(csvCanDump, ",%d\n", frame->dataLength);
            
        }
        busyFileCan = false;
    }
    
    void openFiles() {
        time_t t = time(NULL);
        struct tm tm = *localtime(&t);
        
        char mkdirCommand[1024];
//        std::stringstream newFileName;
//        newFileName << dataDirectory;
//        newFileName << timeStamp;
        
        std::stringstream newDirectoryName;
        sprintf(timeStamp, "/%04d_%02d_%02d/", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
        newDirectoryName << dataDirectory;
        newDirectoryName << timeStamp;
        
        sprintf(mkdirCommand, "mkdir -p %s", newDirectoryName.str().c_str());
        system(mkdirCommand);
        
        sprintf(timeStamp, "%04d-%02d-%02d-%02d-%02d-%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour + tm.tm_isdst, tm.tm_min, tm.tm_sec);
        
        std::stringstream newFileName;
        newFileName << newDirectoryName.str();
        newFileName << timeStamp;
        newFileName << "-GPS.csv";
        gpsFilename = newFileName.str();
        
        newFileName.str(std::string());
        newFileName << newDirectoryName.str();
        newFileName << timeStamp;
        newFileName << "-CAN.csv";
        canFilename = newFileName.str();
        
        std::cout << "GPS filename: " << gpsFilename << std::endl;
        std::cout << "CAN filename: " << canFilename << std::endl;
        
        openGpsFile();
        openCanFile();
    }
    
    void closeFiles() {
        closeCanFile();
        closeGpsFile();
    }
    
    
    const char* stateToStr(RecordingState& state) {
        switch (state) {
            case STATE_IDLE: return "STATE_IDLE";
            case STATE_RECORDING: return "STATE_RECORDING";
        }
        return "";
    }
    
    void transitionToState(RecordingState newState) {
        std::cout << "INFO: CsvRecorder: Transitioning from state " << stateToStr(state) << " -> " << stateToStr(newState) << std::endl;
        exitAction();
        state = newState;
        entryAction();
    }
    
    void entryAction() {
        switch (state) {
            case STATE_IDLE:
                if(beepNotification) {
                    std::cout << "BEEP BEEP!!" << std::endl;
                    beeper.doubleBeep();
                }
                break;
                
        case STATE_RECORDING:
                openFiles();
                if(beepNotification) {
                    std::cout << "BEEP BEEP BEEP!!!" << std::endl;
                    beeper.tripleBeep();
                }
                break;
        }
    }
    void doAction() {
        
    }
    void exitAction() {
        switch (state) {
            case STATE_IDLE:
                break;
                
        case STATE_RECORDING:
                closeFiles();
                break;
        }
        
    }
    
    
    
public:
    CsvRecorder( ) {
        state = STATE_IDLE;
        
        busyFileGps = false;
        busyFileCan = false;
        
        beepNotification = false;
//        recordingAllowed = false;
        recordingEnabled = false;
        
        zoneChecker = NULL;
    }
    ~CsvRecorder( ) {
        transitionToState(STATE_IDLE);
        
    }
    
    void setZoneChecker(ZoneChecker* checker) {
        zoneChecker = checker;
    }
    
    void enableBeep() {
        beepNotification = true;
    }
    
//    void saveToCsvFiles(const char* gps, const char* can) {
    void saveToDirectory(const char* path) {
//        gpsFilename = gps;
//        canFilename = can;
        
        dataDirectory = path;
        std::cout << "Provided path: " << dataDirectory << std::endl;
         if(dataDirectory.size() > 1 && dataDirectory[dataDirectory.size()-1] == '/') {
            dataDirectory.pop_back();
        }
        std::cout << "New path: " << dataDirectory << std::endl;
        
        recordingEnabled = true;
        
//        recordingEnabled = true;
//        if(recordingAllowed) {
//            openFiles();
//        }
    }
    
    
};

using namespace std;
int main(int argc, char **argv) {
    
    ZoneChecker zCheck;
//    zCheck.open("/etc/libpanda.d/zone-testbed.json");
    if(zCheck.open("/etc/libpanda.d/zone.json")) {
        std::cout << "Warning!  No zonefile defined, using default zonefile instead..." << std::endl;
        zCheck.open("/etc/libpanda.d/zone-default.json");
    }
    zCheck.save("/etc/libpanda.d/zone-processed.json");
    
//    Vertex point1;
//    point1.x = 10.5;
//    point1.y = 0.5;
//    std::cout << std::endl << std::endl;
//    std::cout << zCheck.inHysteresisZone(point1) << " ";
//    point1.x = 1.5;
//    point1.y = 0.5;
//    std::cout <<  zCheck.inHysteresisZone(point1) << " ";
//    point1.x = 0.5;
//    point1.y = 0.5;
//    std::cout << zCheck.inHysteresisZone(point1) << std::endl;
    
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
//	const char*    gpsFilename = NULL;
	const char*   nmeaFilename = NULL;
//	const char* canCsvFilename = NULL;
    const char* dataDirectory = NULL;
	const char* canRawFilename = NULL;
    bool beepOnRecordChange = false;
	int ch;
	while ((ch = getopt_long(argc, argv, "u:d:vfb", long_options, NULL)) != -1)
	{
		switch (ch)
		{
			case 'u':
				switch (optarg[0]) {
					case 'a': usbMode = Panda::MODE_ASYNCHRONOUS; break;
					case 's': usbMode =  Panda::MODE_SYNCHRONOUS; break;
					case 'i': usbMode =  Panda::MODE_ISOCHRONOUS; break; };
				break;
//			case 'g':    gpsFilename = optarg; break;
//			case 'n':   nmeaFilename = optarg; break;
//			case 'c': canCsvFilename = optarg; break?;
//			case 'r': canRawFilename = optarg; break;
            case 'd': dataDirectory = optarg; break;
			case 'f':  forceNissan = true; break;
            case 'b': beepOnRecordChange = true; break;
			default:
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
				break;
		}
	}
    
    if(beepOnRecordChange) {
        mCsvRecorder.enableBeep();
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
//				std::cerr << ".";
                printTenThenNewLine(".");
                
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


//	if (gpsFilename != NULL && canCsvFilename != NULL) {
////		pandaHandler.getGps().saveToCsvFile(gpsFilename);
////        myGpsObserver.saveToCsvFile(gpsFilename);
//        mCsvRecorder.saveToCsvFiles(gpsFilename, canCsvFilename);
//	}
    
    if( dataDirectory != NULL ) {
        mCsvRecorder.saveToDirectory(dataDirectory);
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
//			std::cerr << ".";
            printTenThenNewLine(".");
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
