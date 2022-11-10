//
//  createObject.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/7.
//

#include <math.h>
#include "createObject.hpp"
#include "utility.h"

int numPoints = 0;
int numSprings = 0;
struct Point points[MAXN];
struct Spring springs[MAXN_SQR];

const bool breathing = false;
const double OMEGA = 3.1415926;
const double b = 0.05;

double calcDist(double p1[3], double p2[3]);
void initializePointsCube();
void initializePointsTetrahedral();
void initializeSprings();
int initializePointsTwoCubes();
void initializePointsWalkingCube();
void initializeSpringsForEachCube();

void initializeCube()
{
    numPoints = 8;
    initializePointsCube();
    initializeSprings();
}

void initializeTetrahedral()
{
    numPoints = 4;
    initializePointsTetrahedral();
    initializeSprings();
}

void initializeTwoCubes()
{
    numPoints = 12;
    initializePointsTwoCubes();
    initializeSpringsForEachCube();
}

void initializeWalkingCubes()
{
    numPoints = 115;
    initializePointsWalkingCube();
    initializeSpringsForEachCube();
}

void initializePointsCube()
{
    for(int i = 0; i < numPoints; i ++)
    {
        points[i].mass = 0.1;
        for(int j = 0; j < 3; j ++)
        {
            points[i].velocity[j] = 0;
            points[i].accel[j] = 0;
            points[i].force[j] = 0;
        }
    }
    
//    float x[4] = {0, 0.6, 0.8, 1.4};
//    float z[4] = {0.6, 1.4, 0, 0.8};
    float x[4] = {0, 1, 0, 1};
    float z[4] = {0, 0, 1, 1};
    
    float drop_height = 2;
    
    int p = 0;
    for(int i = 0; i < 4; i ++)
    {
        for(int j = 0; j <= 1; j ++)
        {
            points[p].pos[0] = x[i] * 1;
            points[p].pos[1] = j * 1;
            points[p].pos[2] = z[i] * 1 + drop_height;
            p ++;
        }
    }
}

void initializePointsTetrahedral()
{
    numPoints = 4;
    for(int i = 0; i < numPoints; i ++)
    {
        points[i].mass = 0.1;
        for(int j = 0; j < 3; j ++)
        {
            points[i].velocity[j] = 0;
            points[i].accel[j] = 0;
            points[i].force[j] = 0;
        }
    }
    
    double h = sqrt(3) * 0.5;
    double hmid = h / 3;
    double x[4] = {0, 0.5, 1, 0.5};
    double y[4] = {0, h, 0, hmid};
    double z[4] = {0, 0, 0, h};
    
    double drop_height = 0.2;
    
    int p = 0;
    for(int i = 0; i < 4; i ++)
    {
        points[p].pos[0] = x[i] * 0.1;
        points[p].pos[1] = y[i] * 0.1;
        points[p].pos[2] = z[i] * 0.1 + drop_height;
        p ++;
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
            
            if(breathing)
            {
                springs[p].muscle = true;
                springs[p].b = b;
                springs[p].omega = OMEGA;
                springs[p].c = -0.0002;
            }
            p ++;
        }
    }
    numSprings = p;
}

int initializePointsTwoCubes()
{
    for(int i = 0; i < numPoints; i ++)
    {
        points[i].mass = 0.1;
        for(int j = 0; j < 3; j ++)
        {
            points[i].velocity[j] = 0;
            points[i].accel[j] = 0;
            points[i].force[j] = 0;
        }
    }
    
    double x[4] = {0.0, 1.0, 2.0};
    
    int p = 0;
    for(int i = 0; i < 3; i ++)
    {
        for(int y = 0; y <= 1; y ++)
        {
            for(int z = 0; z <= 1; z ++)
            {
                points[p].pos[0] = x[i];
                points[p].pos[1] = y;
                points[p].pos[2] = z;
                p ++;
            }
        }
    }
    
    return p;
}

int initializeFeet(struct Point* points_start, int z)
{
    double x[4] = {0, 1, 3, 4};
    double y[5] = {0, 1, 2, 3, 4};
    int p = 0;
    
    for(int i = 0; i < 4; i ++)
    {
        for(int j = 0; j < 5; j ++)
        {
            
            points_start[p].pos[0] = x[i];
            points_start[p].pos[1] = y[j];
            points_start[p].pos[2] = z;
            
            p ++;
        }
    }
    
    return p;
}

int initializeBody(struct Point* points_start, int z)
{
    double x[5] = {0, 1, 2, 3, 4};
    double y[5] = {0, 1, 2, 3, 4};
    int p = 0;
    
    for(int i = 0; i < 5; i ++)
    {
        for(int j = 0; j < 5; j ++)
        {
            
            points_start[p].pos[0] = x[i];
            points_start[p].pos[1] = y[j];
            points_start[p].pos[2] = z;
            
            p ++;
        }
    }
    
    return p;
}

void initializePointsWalkingCube()
{
    numPoints = 115;
    int p = 0;
    
    for(int z = 0; z <= 4; z ++)
    {
        if(z == 0 || z == 1)
        {
            p += initializeFeet(&points[p], z);
        }
        else
        {
            p += initializeBody(&points[p], z);
        }
    }
    
    for(int i = 0; i < numPoints; i ++)
    {
        points[i].mass = 0.1;
        
        for(int j = 0; j < 3; j ++)
        {
            points[i].velocity[j] = 0;
            points[i].accel[j] = 0;
            points[i].force[j] = 0;
        }
    }
    
    return;
}

void initializeSpringsForEachCube()
{
    int p = 0;
    
    for(int i = 0; i < numPoints - 1; i ++)
    {
        for(int j = i + 1; j < numPoints; j ++)
        {
            if(calcDist(points[i].pos, points[j].pos) > sqrt(3) + 0.01)
            {
                continue;
            }
            
            springs[p].p1 = &points[i];
            springs[p].p2 = &points[j];
            springs[p].len = calcDist(points[i].pos, points[j].pos);
            springs[p].k = 10000;
            p ++;
        }
    }
    
    numSprings = p;
    
    return;
}

double calcDist(double p1[3], double p2[3])
{
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2));
}