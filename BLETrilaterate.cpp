#include "BLETrilaterate.h"
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
    beacon = new BLELocBeacon();
    _beaconList = LinkedList<BLELocBeacon*>();
    _pos[0] = 0;
    _pos[1] = 0;
}

void BLETrilaterate::addBeacon(BLELocBeacon *beacon){
    refreshBeaconList();
    _beaconList.add(beacon);
    //_beaconList.sort(compare);
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

void BLETrilaterate::refreshBeaconList(){
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if (millis() - _beaconList.get(i)->getLastSeen() > BEACON_TIMEOUT ){
            _beaconList.remove(i);
        }
    }
}

int BLETrilaterate::getClosetBeaconIndex(){
    int cur_rssi = -999;
    int index = 0;
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if(_beaconList.get(i)->getRSSI() > cur_rssi){
            cur_rssi = _beaconList.get(i)->getRSSI();
            index = i;
        }
    }
    return index;
}

void BLETrilaterate::updateBeaconList(char id, float rssi){
    refreshBeaconList();
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if (_beaconList.get(i)->getID() == id){
           _beaconList.get(i)->setRSSI(rssi);
           _beaconList.get(i)->setLastSeen(millis());
        }
    }
    _beaconList.sort(compare);
}

void BLETrilaterate::getBeaconByIndex(int index, BLELocBeacon *beacon){
    *beacon = *_beaconList.get(index);
}

void BLETrilaterate::getBeaconByID(char id, BLELocBeacon *beacon){
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if (_beaconList.get(i)->getID() == id){
            *beacon = *_beaconList.get(i);
        }
    }
}

float BLETrilaterate::getBeaconRSSI(char id){
    for (int i = 0 ; i < _beaconList.size() ; i++){
        if (_beaconList.get(i)->getID() == id){
            return _beaconList.get(i)->getRSSI();
        }
    }
    return -999;
}

void BLETrilaterate::getClosestBeacon(BLELocBeacon *beacon){
    *beacon = *_beaconList.get(0);
}

void BLETrilaterate::getFurthestBeacon(BLELocBeacon *beacon){
    *beacon = *_beaconList.get(_beaconList.size() - 1);
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

        //ESP_LOGI("TRI", "distance 3: %f", _beaconList.get(3)->getDistanceSq() );
        a_mat << 2 * (_beaconList.get(0)->getFixedPos_X() - _beaconList.get(3)->getFixedPos_X()), 2 * ((_beaconList.get(1)->getFixedPos_Y() - _beaconList.get(3)->getFixedPos_Y())),
                 2 * (_beaconList.get(1)->getFixedPos_X() - _beaconList.get(3)->getFixedPos_X()), 2 * ((_beaconList.get(1)->getFixedPos_Y() - _beaconList.get(3)->getFixedPos_Y())),
                 2 * (_beaconList.get(2)->getFixedPos_X() - _beaconList.get(3)->getFixedPos_X()), 2 * ((_beaconList.get(2)->getFixedPos_Y() - _beaconList.get(3)->getFixedPos_Y()));

        b_mat << _beaconList.get(3)->getDistanceSq() - _beaconList.get(0)->getDistanceSq() +
                 _beaconList.get(0)->getFixedPosSq_X() - _beaconList.get(3)->getFixedPosSq_X() + _beaconList.get(0)->getFixedPosSq_Y() - _beaconList.get(3)->getFixedPosSq_Y(),

                 _beaconList.get(3)->getDistanceSq() - _beaconList.get(1)->getDistanceSq() +
                 _beaconList.get(1)->getFixedPosSq_X() - _beaconList.get(3)->getFixedPosSq_X() + _beaconList.get(1)->getFixedPosSq_Y() - _beaconList.get(3)->getFixedPosSq_Y(),

                 _beaconList.get(3)->getDistanceSq() - _beaconList.get(2)->getDistanceSq() +
                 _beaconList.get(2)->getFixedPosSq_X() - _beaconList.get(3)->getFixedPosSq_X() + _beaconList.get(2)->getFixedPosSq_Y() - _beaconList.get(3)->getFixedPosSq_Y();

        temp_mat = ((a_mat).transpose() * a_mat);
        x_mat = temp_mat.inverse() * (a_mat.transpose() * b_mat);


        // Serial.println("A matrix");
        // print_mtxf(a_mat);
        // Serial.println("");
        // Serial.println("B matrix");
        // print_mtxf(b_mat);
        // Serial.println("");
        // Serial.println("X Matrix");
        // print_mtxf(x_mat);
        // Serial.println("");
        // _pos[0] =  x_mat(0,0);
        // _pos[1] = x_mat(1,0);
        // position[0] = x_mat(0,0);
        // position[1] = x_mat(1,0);

        //ESP_LOGI("TRI", "x : %f , y: %f", _pos[0], _pos[1]);

    }

    // else{
    //     getClosestBeacon(beacon);
    //     position[0] = beacon->getFixedPos_X();
    //     position[1] = beacon->getFixedPos_Y();

    // }
}

float BLETrilaterate::getestimatedPositionX(){
    return _pos[0];
}

float BLETrilaterate::getestimatedPositionY(){
    return _pos[1];
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