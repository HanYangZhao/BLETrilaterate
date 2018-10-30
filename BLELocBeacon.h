#ifndef __BLE_LOC_BEACON_H__
#define __BLE_LOC_BEACON_H__

#include <stdint.h>
#include <math.h>
#include "SimpleKalmanFilter.h"

class BLELocBeacon{
    public:
        BLELocBeacon();
        BLELocBeacon(char id, float x, float y, unsigned long lastSeen);
        BLELocBeacon(char id, float x, float y, unsigned long lastSeen, float rssi, int16_t measuredTX, uint8_t pathLossExp);
        float getFixedPos_X();
        float getFixedPosSq_X();
        float getFixedPos_Y();
        float getFixedPosSq_Y();
        long getLastSeen();
        char getID();
        void setFixedPos_X(float x);
        void setFixedPos_Y(float y);
        void setLastSeen(unsigned long timestamp);
        float getRSSI();
        int setRSSI(float rssi);
        float getDistance();
        float getDistanceSq();
        void setDistance(float distance);
    private:
        char _id;
        float _x;
        float _y;
        unsigned long _timestamp;
        float _rssi;
        float _distance;
        int16_t _measuredTX;
        uint8_t _pathLossExp;
        SimpleKalmanFilter _kalmanRssi = SimpleKalmanFilter(1.4,100, 0.065);

};
#endif