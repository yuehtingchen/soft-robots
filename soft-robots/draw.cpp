//
//  draw.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/10/21.
//

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <unistd.h>
using namespace std;

#include "setPoints.hpp"
#include "utility.h"

float const GRAVITY[3] = {0, 0, -9.81};
float const TIME_STEP = 0.0001;
int const kc = 100000; // restoration force
float const MAX_TIME = 1;
float T = 0;
int numPoints = 8;
int numSprings = numPoints * (numPoints - 1) / 2;
struct Point points[MAXN];
struct Spring springs[MAXN];

void printPoints();
void calcForce();
void updatePointsPos();
void resetPointForce();
void calcSpringForce();
void calcGravitationalForce();
void calcRestorationForce();
float calcDist(float p1[3], float p2[3]);
void calcVector(float vec[3], float p1[3], float p2[3]);
void normalizeVector(float vec[3]);
void reverseVector(float vec[3]);
void initializePoints();
void initializeSprings();

void updatePoints()
{
    printPoints();
    calcForce();
    updatePointsPos();
    T += TIME_STEP;
    
    sleep(1);
    
    return;
}

void printPoints()
{
    for(int i = 0; i < numPoints; i ++)
    {
        printf("%f %f %f\n", points[i].pos[0], points[i].pos[1], points[i].pos[2]);
        printf("F: %f %f %f\n", points[i].force[0], points[i].force[1], points[i].force[2]);
        printf("\n");
    }
    printf("--------------------\n");
}

void calcForce()
{
    resetPointForce();
    calcSpringForce();
    calcGravitationalForce();
    calcRestorationForce();
    
    return;
}

void updatePointsPos()
{
    for(int i = 0; i < numPoints; i ++)
    {
        for(int j = 0; j < 3; j ++)
        {
            points[i].accel[j] = points[i].force[j] / points[i].mass;
            points[i].velocity[j] += points[i].accel[j] * TIME_STEP;
            points[i].pos[j] += points[i].velocity[j] * TIME_STEP;
        }
    }
}

void resetPointForce()
{
    for(int i = 0; i < numPoints; i ++)
    {
        points[i].force[0] = 0;
        points[i].force[1] = 0;
        points[i].force[2] = 0;
    }
    
    return;
}

void calcSpringForce()
{
    for(int i = 0; i < numSprings; i ++)
    {
        struct Point *p1 = springs[i].p1;
        struct Point *p2 = springs[i].p2;
        float k = springs[i].k;
        float len = springs[i].len;
        
        float dist = calcDist(p1->pos, p2->pos);
        float force = k * (dist - len);
        float vec_p1_p2[3];
        calcVector(vec_p1_p2, p1->pos, p2->pos);
        normalizeVector(vec_p1_p2);
        
        for(int i = 0; i < 3; i ++)
        {
            p1->force[i] += force * vec_p1_p2[i];
        }
        
        reverseVector(vec_p1_p2);
        
        for(int i = 0; i < 3; i ++)
        {
            p2->force[i] += force * vec_p1_p2[i];
        }
    }
}

void calcGravitationalForce()
{
    for(int i = 0; i < numPoints; i ++)
    {
        for(int j = 0; j < 3; j ++)
        {
            points[i].force[j] += points[i].mass * GRAVITY[j];
        }
    }
    
    return;
}

void calcRestorationForce()
{
    for(int i = 0; i < numPoints; i ++)
    {
        if(points[i].pos[2] >= 0) continue;
        
        float force[3] = {0, 0, -1 * kc * points[i].pos[2]};
        for(int j = 0; j < 3; j ++)
        {
            points[i].force[j] += force[j];
        }
    }
    
    return;
}

float calcDist(float p1[3], float p2[3])
{
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2));
}

void calcVector(float vec[3], float p1[3], float p2[3])
{
    vec[0] = p2[0] - p1[0];
    vec[1] = p2[1] - p1[1];
    vec[2] = p2[2] - p1[2];
    
    return;
}

void normalizeVector(float vec[3])
{
    float denom = sqrt(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2));
    for(int i = 0; i < 3; i ++)
    {
        vec[i] = vec[i] / denom;
    }
    
    return;
}

void reverseVector(float vec[3])
{
    for(int i = 0; i < 3; i ++)
    {
        vec[i] = vec[i] * -1;
    }
    
    return;
}

void initializePoints()
{
    for(int i = 0; i < numPoints; i ++)
    {
        points[i].mass = 0.1;
        points[i].velocity[0] = points[i].velocity[1] = points[i].velocity[2] = 0;
        points[i].accel[0] = points[i].accel[1] = points[i].accel[2] = 0;
    }
    
    int p = 0;
    for(int i = 0; i <= 1; i ++)
    {
        for(int j = 2; j <= 3; j ++)
        {
            for(int k = 0; k <= 1; k ++)
            {
                points[p].pos[0] = i;
                points[p].pos[1] = j;
                points[p].pos[2] = k;
                p ++;
            }
        }
    }
}

void initializeSprings()
{
    int p = 0;
    for(int i = 0; i < numPoints; i ++)
    {
        for(int j = i + 1; j < numPoints; j ++)
        {
            springs[p].p1 = &points[i];
            springs[p].p2 = &points[j];
            springs[p].len = calcDist(points[i].pos, points[j].pos);
            springs[p].k = 10000;
            p ++;
        }
    }
}
