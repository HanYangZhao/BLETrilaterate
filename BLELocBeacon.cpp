#include "BLELocBeacon.h"

static float calculateDistance(int8_t rssi, int8_t measuredTX);

BLELocBeacon::BLELocBeacon(){}

BLELocBeacon::BLELocBeacon(char id, float x, float y,  unsigned long lastSeen){
    _id = id;
    _x = x;
    _y = y;
    _timestamp = lastSeen;
    _rssi = -999;
    _distance = 999;
}

BLELocBeacon::BLELocBeacon(char id, float x, float y, unsigned long lastSeen, float rssi, uint8_t measuredTX){
    _id = id;
    _x = x;
    _y = y;
    _timestamp = lastSeen;
    _rssi = rssi;
    _measuredTX = measuredTX;
    _distance = calculateDistance(rssi,measuredTX);

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

float BLELocBeacon::getFixedPosSqrt_X(){
    return _x*_x;
}

float BLELocBeacon::getFixedPos_Y(){
    return _y;
}

float BLELocBeacon::getFixedPosSqrt_Y(){
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

float BLELocBeacon::getDistanceSqrt(){
    return _distance * _distance;
}

float BLELocBeacon::getRSSI(){
    return _rssi;
}

void BLELocBeacon::setRSSI(float rssi){
    _rssi = rssi;
    _distance = calculateDistance(rssi,_measuredTX);
}

static float calculateDistance(int8_t rssi, int8_t measuredTX){
    return pow(10, float((measuredTX - rssi)) / (10 * PATH_LOSS_EXP));
}