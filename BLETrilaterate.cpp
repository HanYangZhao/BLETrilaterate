#include "BLETrilaterate.h"
#include "Arduino.h"
#define BEACON_TIMEOUT 30000 //ms


static MatrixXf a_mat(3,2);
static MatrixXf b_mat(3,1);
static MatrixXf x_mat(2,1);
static MatrixXf temp_mat(2,2);
static void print_mtxf(const Eigen::MatrixXf& X);

static int compare(BLELocBeacon *&a, BLELocBeacon *&b){
    if (a->getDistance() < b->getDistance())
        return -1;
    return 1;
}

BLETrilaterate::BLETrilaterate(){
    _beaconList = LinkedList<BLELocBeacon*>();
    _pos[0] = 0;
    _pos[1]= 0;
}

void BLETrilaterate::addBeacon(BLELocBeacon *beacon){
    _beaconList.add(beacon);
    _beaconList.sort(compare);
    // for (int i = 0 ; i < _beaconList.size() ; i++){
    //     Serial.println(_beaconList.get(i)->getDistance());
    // }
    // Serial.println("");
}

void BLETrilaterate::removeBeacon(char id){
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if (_beaconList.get(i)->getID() == id){
            _beaconList.remove(i);
        }
    }
}

void BLETrilaterate::clearBeacons(){
    _beaconList.clear();
}

void BLETrilaterate::refreshBeaconList(unsigned long timestamp){
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if (timestamp - _beaconList.get(i)->getLastSeen() > BEACON_TIMEOUT ){
            _beaconList.remove(i);
        }
    }
}

void BLETrilaterate::updateBeaconList(char id, float rssi){
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if (_beaconList.get(i)->getID() == id){
            _beaconList.get(i)->setRSSI(rssi);
        }
    }
    _beaconList.sort(compare);
}

void BLETrilaterate::getBeacon(int index, BLELocBeacon *beacon){
    beacon = _beaconList.get(index);
}

void BLETrilaterate::getClosestBeacon(BLELocBeacon *beacon){
    beacon = _beaconList.get(0);
}

void BLETrilaterate::getFurthestBeacon(BLELocBeacon* beacon){
    beacon = _beaconList.get(_beaconList.size() - 1);
}

bool BLETrilaterate::isExistingBeacon(char id){
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if (_beaconList.get(i)->getID() == id){
            return true;
        }
    }
    return false;
}

void BLETrilaterate::estimatePosition(float (&position)[2]){
    if (_beaconList.size() > 3){

        a_mat << 2 * (_beaconList.get(0)->getFixedPos_X() - _beaconList.get(3)->getFixedPos_X()), 2 * ((_beaconList.get(1)->getFixedPos_Y() - _beaconList.get(3)->getFixedPos_Y())),
                 2 * (_beaconList.get(1)->getFixedPos_X() - _beaconList.get(3)->getFixedPos_X()), 2 * ((_beaconList.get(1)->getFixedPos_Y() - _beaconList.get(3)->getFixedPos_Y())),
                 2 * (_beaconList.get(2)->getFixedPos_X() - _beaconList.get(3)->getFixedPos_X()), 2 * ((_beaconList.get(2)->getFixedPos_Y() - _beaconList.get(3)->getFixedPos_Y()));

        b_mat << _beaconList.get(3)->getDistanceSqrt() - _beaconList.get(0)->getDistanceSqrt() +
                 _beaconList.get(0)->getFixedPosSqrt_X() - _beaconList.get(3)->getFixedPosSqrt_X() + _beaconList.get(0)->getFixedPosSqrt_Y() - _beaconList.get(3)->getFixedPosSqrt_Y(),

                 _beaconList.get(3)->getDistanceSqrt() - _beaconList.get(1)->getDistanceSqrt() +
                 _beaconList.get(1)->getFixedPosSqrt_X() - _beaconList.get(3)->getFixedPosSqrt_X() + _beaconList.get(1)->getFixedPosSqrt_Y() - _beaconList.get(3)->getFixedPosSqrt_Y(),

                 _beaconList.get(3)->getDistanceSqrt() - _beaconList.get(2)->getDistanceSqrt() +
                 _beaconList.get(2)->getFixedPosSqrt_X() - _beaconList.get(3)->getFixedPosSqrt_X() + _beaconList.get(2)->getFixedPosSqrt_Y() - _beaconList.get(3)->getFixedPosSqrt_Y();
        
        temp_mat = ((a_mat).transpose() * a_mat);
        x_mat = temp_mat.inverse() * (a_mat.transpose() * b_mat);


        // Serial.println("A matrix");
        // print_mtxf(a_mat);
        // Serial.println("");
        // Serial.println("B matrix");
        // print_mtxf(b_mat);
        // Serial.println("");
        // Serial.println("X Matrix");
        //print_mtxf(x_mat);
        // Serial.println("");

        _pos[0] =  x_mat(0,0);
        _pos[1] = x_mat(1,0);
        position[0] = x_mat(0,0);
        position[1] = x_mat(1,0);

    }
}


// void print_mtxf(const Eigen::MatrixXf& X)  
// {
//    int i, j, nrow, ncol;
   
//    nrow = X.rows();
//    ncol = X.cols();

//    Serial.print("nrow: "); Serial.println(nrow);
//    Serial.print("ncol: "); Serial.println(ncol);       
//    Serial.println();
   
//    for (i=0; i<nrow; i++)
//    {
//        for (j=0; j<ncol; j++)
//        {
//            Serial.print(X(i,j), 6);   // print 6 decimal places
//            Serial.print(", ");
//        }
//        Serial.println();
//    }
//    Serial.println();
// }