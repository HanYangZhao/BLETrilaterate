
#include "BLETrilaterate.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
static const char* TAG = "EDDYSTONE_DEMO";
#include "Arduino.h"
#include "math.h"

BLETrilaterate trilaterate = BLETrilaterate();
float real_x = 5;
float real_y = 2;
float position[2];

BLELocBeacon *bec_4 = new BLELocBeacon(1,10.0,3.0,0);
BLELocBeacon *bec_3 = new BLELocBeacon(2,2.0,2.0,0);
BLELocBeacon *bec_2 = new BLELocBeacon(3,10.0,7.9,0);
BLELocBeacon *bec_1 = new BLELocBeacon(4,2.0,9.0,0);


void setup(){
    Serial.begin(9600);
    Serial.println("setup");

    bec_1->setDistance(sqrt(pow((real_x - bec_1->getFixedPos_X()),2) + pow(real_y - bec_1->getFixedPos_Y(),2)));
    bec_2->setDistance(sqrt(pow((real_x - bec_2->getFixedPos_X()),2) + pow(real_y - bec_2->getFixedPos_Y(),2)));
    bec_3->setDistance(sqrt(pow((real_x - bec_3->getFixedPos_X()),2) + pow(real_y - bec_3->getFixedPos_Y(),2)));
    bec_4->setDistance(sqrt(pow((real_x - bec_4->getFixedPos_X()),2) + pow(real_y - bec_4->getFixedPos_Y(),2)));

}

void loop() {

    Serial.println("");
    trilaterate.addBeacon(bec_1);
    trilaterate.addBeacon(bec_2);
    trilaterate.addBeacon(bec_3);
    trilaterate.addBeacon(bec_4);

    
    trilaterate.estimatePosition(position);

    Serial.println(position[0]);
    Serial.println(position[1]);
    delay(100);
    trilaterate.clearBeacons();

    Serial.println("");
}
