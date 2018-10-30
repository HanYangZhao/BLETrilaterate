#include "BLELocBeacon.h"
#include <esp_log.h>

static float calculateDistance(float rssi, int16_t measuredTX, uint8_t pathLossExp){
    return pow(10, float((measuredTX - rssi)) / (10 * pathLossExp));
}

BLELocBeacon::BLELocBeacon(){
    _id = 0;
    _x = 0;
    _y = 0;
    _timestamp = 0;
    _rssi = -999;
    _distance = 0;
    _pathLossExp = 2;
}

BLELocBeacon::BLELocBeacon(char id, float x, float y,  unsigned long lastSeen){
    _id = id;
    _x = x;
    _y = y;
    _timestamp = lastSeen;
    _rssi = -999;
    _distance = 0;
}

BLELocBeacon::BLELocBeacon(char id, float x, float y, unsigned long lastSeen, float rssi, int16_t measuredTX, uint8_t pathLossExp){
    _id = id;
    _x = x;
    _y = y;
    _timestamp = lastSeen;
    _rssi = rssi;
    _measuredTX = measuredTX;
    _distance = calculateDistance(rssi,measuredTX,pathLossExp);
    _pathLossExp = pathLossExp;
}

void BLELocBeacon::setFixedPos_X(float x){
    _x = x;
}

void BLELocBeacon::setFixedPos_Y(float y){
    _y = y;
}

void BLELocBeacon::setLastSeen(unsigned long timestamp){
    _timestamp = timestamp;
}

float BLELocBeacon::getFixedPos_X(){
    return _x;
}

float BLELocBeacon::getFixedPosSq_X(){
    return _x*_x;
}

float BLELocBeacon::getFixedPos_Y(){
    return _y;
}

float BLELocBeacon::getFixedPosSq_Y(){
    return _y*_y;
}

char BLELocBeacon::getID(){
    return _id;
}

long BLELocBeacon::getLastSeen(){
    return _timestamp;
}

void BLELocBeacon::setDistance(float distance){
    _distance = distance;
}

float BLELocBeacon::getDistance(){
    return _distance;
}

float BLELocBeacon::getDistanceSq(){
    return _distance * _distance;
}

float BLELocBeacon::getRSSI(){
    return _rssi;
}

int BLELocBeacon::setRSSI(float rssi){
    _rssi = _kalmanRssi.updateEstimate(rssi);
    _distance = calculateDistance(_rssi,_measuredTX,_pathLossExp);
    return 0;
}