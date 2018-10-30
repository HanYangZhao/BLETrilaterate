#ifndef __BLETrilaterate_H__
#define __BLETrilaterate_H__


#include "esp32-hal.h"
#include "math.h"
#include "LinkedList.h"
#include "BLELocBeacon.h"
#include <stdint.h>
#include <Eigen.h>     // Calls main Eigen matrix class library
#include <Eigen/LU>             // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

class BLETrilaterate
{
    public:
        BLETrilaterate();
        void addBeacon(BLELocBeacon *beacon);
        void removeBeacon(char id);
        void clearBeacons();
        void refreshBeaconList();
        void refreshBeaconList(unsigned long timestamp);
        void updateBeaconList(char id, float rssi);
        bool isExistingBeacon(char id);
        void getBeaconByIndex(int index, BLELocBeacon *beacon);
        void getBeaconByID(char id, BLELocBeacon *beacon);
        float getBeaconRSSI(char id);
        void getBeaconList(LinkedList<BLELocBeacon> *b_list);
        void getClosestBeacon(BLELocBeacon *beacon);
        void getFurthestBeacon(BLELocBeacon *beacon);
        void estimatePosition(float (&position)[2]);
        float getestimatedPositionX();
        float getestimatedPositionY();
    private:
        LinkedList<BLELocBeacon*> _beaconList;
        float _pos[];
        BLELocBeacon *beacon;
        int getClosetBeaconIndex();
};

#endif