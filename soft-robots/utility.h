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
    float mass;
    float pos[3];
    float velocity[3];
    float accel[3];
    float force[3];
};

/* length = len + b * sin(omega * T + c) */
struct Spring
{
    struct Point* p1;
    struct Point* p2;
    float len;
    float k;
    bool muscle = false;
    float omega;
    float b;
    float c;
};

struct Material
{
    struct Point* p;
    float len;
    float k;
    bool muscle = false;
    float omega;
    float b;
    float c;
};

#endif /* utility_h */
