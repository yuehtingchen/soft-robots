//
//  utility.h
//  soft-robots
//
//  Created by Alice Chen on 2022/10/29.
//

#ifndef utility_h
#define utility_h

#define MAXN 100

struct Point
{
    float mass;
    float pos[3];
    float velocity[3];
    float accel[3];
    float force[3];
};

struct Spring
{
    struct Point* p1;
    struct Point* p2;
    float len;
    float k;
};

#endif /* utility_h */
