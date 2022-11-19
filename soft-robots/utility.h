//
//  utility.h
//  soft-robots
//
//  Created by Alice Chen on 2022/10/29.
//

#ifndef utility_h
#define utility_h

#define MAXN 150
#define MAXN_SQR 2000

const int MAX_SIDE = 4;
const int MAX_SIDE_1 = MAX_SIDE + 1;
const int MAX_BLOCKS = MAX_SIDE * MAX_SIDE * MAX_SIDE;
const int MAX_POINTS = (MAX_SIDE + 1) * (MAX_SIDE + 1) * (MAX_SIDE + 1);

const int testNum = 1;
const int evaluationTimes = 100;
const int selectInterval = 5;
const int sampleSize = 10;

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
    double omega;
    double b;
    double c;
    
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
};

#endif /* utility_h */
