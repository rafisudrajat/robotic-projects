#ifndef _BEHAV_LOMBA1
#define _BEHAV_LOMBA1

class behav_lomba1{
    private:
        const double setpoint_X, setpoint_Y; //ambil dari orientation_node nilai yawnya
        enum state{
            initial, 
            planning, 
            walking, 
            mini_translation, 
        };
    public:
        state current_state;
        double current_setpoint;
        double calculate_psi(int state, double yawNow); 
        double calculate_theta(int state, double panNow); //panNow tipedatanya nanti diganti yg bener sesuain sm message dari motion manager
        //Implementasi STATE
        behav_lomba1(double set_x, double set_Y);
        void destination_plan(/* diisi objek yang terdeteksi*/); //objek ada 2 yaitu tiang dan sesuatu yang ada di start center
        void walk();
        void mini_translation();

};  