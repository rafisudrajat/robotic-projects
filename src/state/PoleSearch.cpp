#include "../../include/searching.hpp"
#include <ros/ros.h>

POLESEARCH::POLESEARCH() : State("pole_search") {
    objectDetected = false; 
    objectCentered = false;

    poleStartFound = ros::Time::now();
    poleLastFound = ros::Time::now();

    polePos = nh.subscribe<daho_vision::Coordinate>("/pole_detection/pole_position", 1, &Searching::polePosCallback, this);

    sequenceCari.loop = true;
    sequenceCari.reverse = true;

    headSeq = nh.advertise<daho_head::headSequence>("/head/sequence/write", 1);

    // TODO : sequence cari horizontal
} //ctor

POLESEARCH::~POLESEARCH() {

}

void POLESEARCH::process() {
    if (ros::Time::now() - ballLastFound > ros::Duration(5))
    {
        // Sudah lebih dari 5 detik sejak bola terakhir ditemukan
        // Mulai cari bola
        headSeq.publish(sequenceCari);

        objectDetected = false; 
        objectCentered = false;
    }
}

shared_ptr<State> POLESEARCH::checkCond()
{

}

//fungsi buat nentuin posisi objek
void POLESEARCH::polePosCallback(const daho_vision::CoordinateConstPtr &msg)
{
    if(!objectDetected && msg->x > 0 && msg->y > 0)
    {
        poleStartFound = ros::Time::now();
    }
    objectDetected = msg->x > 0 && msg->y > 0;
    bolaDiTengah = abs(msg->x - 320) < 1 && abs(msg->y - 240) < 1; 
    if (adaBola)
    {
        ballLastFound = ros::Time::now();
    }
}