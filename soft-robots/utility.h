//
//  utility.h
//  soft-robots
//
//  Created by Alice Chen on 2022/10/29.
//

#ifndef utility_h
#define utility_h

#define MAXN 1000
#define MAXN_SQR 20000

const int MAX_SIDE = 8;
const int MAX_SIDE_1 = MAX_SIDE + 1;
const int MAX_BLOCKS = MAX_SIDE * MAX_SIDE * MAX_SIDE;
const int MAX_POINTS = (MAX_SIDE + 1) * (MAX_SIDE + 1) * (MAX_SIDE + 1);

const int testNum = 5;
const int evaluationTimes = 100;
const int selectInterval = 5;
const int sampleSize = 10;

const int DIR[6][3] = {
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1},
};

const int CUBE[8][3] = {
    {0, 0, 0},
    {1, 0, 0},
    {0, 1, 0},
    {1, 1, 0},
    {0, 0, 1},
    {1, 0, 1},
    {0, 1, 1},
    {1, 1, 1},
};

struct Point
{
    double mass;
    double pos[3];
    double velocity[3];
    double accel[3];
    double force[3];
};

/* length = len + b * sin(omega * T + c) */
struct Spring
{
    struct Point* p1;
    struct Point* p2;
    double len;
    double k;
    bool muscle = false;
    double omega;
    double b;
    double c;
};

struct Material
{
    double pos[3];
    double len;
    double k;
    bool muscle = false;
    double omega = 0;
    double b = 0;
    double c = 0;
    
    int pIdx()
    {
        return pos[0] + pos[1] * MAX_SIDE_1 + pos[2] * MAX_SIDE_1;
    };
    
    void setPos(int pIdx)
    {
        pos[0] = pIdx % (MAX_SIDE_1);
        pos[1] = pIdx % (MAX_SIDE_1 * MAX_SIDE_1) / MAX_SIDE_1;
        pos[2] = pIdx / MAX_SIDE_1 / MAX_SIDE_1;
    };
    
    Material& operator=(Material other)
    {
        this->pos[0] = other.pos[0];
        this->pos[1] = other.pos[1];
        this->pos[2] = other.pos[2];
        this->k = other.k;
        this->muscle = other.muscle;
        this->omega = other.omega;
        this->b = other.b;
        this->c= other.c;
        return *this;
    }
};

#endif /* utility_h */
