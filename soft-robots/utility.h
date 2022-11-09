//
//  utility.h
//  soft-robots
//
//  Created by Alice Chen on 2022/10/29.
//

#ifndef utility_h
#define utility_h

#define MAXN 10000

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
    struct Point* p;
    double len;
    double k;
    bool muscle = false;
    double omega;
    double b;
    double c;
};

#endif /* utility_h */
