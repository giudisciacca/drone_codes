# include <stdio.h>
# include <math.h>
# include <pico_control/quaternions.cpp>

using namespace std;


struct state{
    vector position;
    vector velocity;
    vector acceleration;
    quaternion q;
}

struct vector{
    float x;
    float y;
    float z;
}

class update_operator(){
    public:
        update_operator(float dt, state drone_state){}

};


class measurement_operator(){};

class EKF(){};

