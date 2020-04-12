#ifndef MOTORS_H
#define MOTORS_H
class Wheels {
    short left_speed;
    short right_speed;
    public:
        void set_speeds(short left, short right);
        short get_left_speed();
        short get_right_speed();
};

class Blades {
    unsigned char left_speed;
    unsigned char right_speed;
    unsigned char trimmer_speed;
    public:
        void enable_blades();
        void disable_blades();
        void set_left_blade(unsigned char left);
        void set_right_blade(unsigned char right);
        void set_trimmer_blade(unsigned char trimmer);
        unsigned char get_left_speed();
        unsigned char get_right_speed();
        unsigned char get_trimmer_speed();
};
#endif
