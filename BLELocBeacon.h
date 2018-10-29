#ifndef __BLE_LOC_BEACON_H__
#define __BLE_LOC_BEACON_H__

#include <stdint.h>
#include <math.h>

#define PATH_LOSS_EXP 3

class BLELocBeacon{
    public:
        BLELocBeacon();
        BLELocBeacon(char id, float x, float y, unsigned long lastSeen);
        BLELocBeacon(char id, float x, float y, unsigned long lastSeen, float rssi, uint8_t measuredTX);
        float getFixedPos_X();
        float getFixedPosSqrt_X();
        float getFixedPos_Y();
        float getFixedPosSqrt_Y();
        long getLastSeen();
        char getID();
        void setFixedPos_X(float x);
        void setFixedPos_Y(float y);
        void setLastSeen(unsigned long timestamp);
        float getRSSI();
        void setRSSI(float rssi);
        float getDistance();
        float getDistanceSqrt();
        void setDistance(float distance);
    private:
        char _id;
        float _x;
        float _y;
        unsigned long _timestamp;
        float _rssi;
        float _distance;
        uint8_t _measuredTX;
};
#endif