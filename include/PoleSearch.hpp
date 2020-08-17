#ifndef POLE_SEARCH_STATE
#define POLE_SEARCH_STATE
#include "state.hpp"

// TODO : include message posisi objek
#include "daho_head/headSequence.h"

class POLESEARCH: public State{    
    private:
        bool objectDetected, objectCentered;
        ros::Time poleStartFound, poleLastFound;
        ros::Publisher headSeq;
        ros::Subscriber polePos;
        daho_head::headSequence sequenceCari;
    
    public :
        SEARCHING(); //ctor
        
        void process();
        
        shared_ptr<State> checkCond();
        
        //fungsi buat nentuin posisi objek
        void polePosCallback(const daho_vision::CoordinateConstPtr &msg);            
}