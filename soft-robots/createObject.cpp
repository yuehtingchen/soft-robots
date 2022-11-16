//
//  createObject.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/7.
//

#include <math.h>
#include <stdio.h>
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
void initializeAllPoints();
void initializePointsCube();
void initializePointsTetrahedral();
void initializePointsTetrahedral(int pIdx, double leftX, double leftY, double leftZ, double yDir, double zDir, double sideLen);
int initializeSprings(int pIdx, int l, int r);
void initializeSprings();
int initializePointsTwoCubes();
void initializePointsWalkingCube();
void initializeInsect();
void initializeSpringsForEachCube();
void initializeSpringsForEachCube(int pIdx, struct Point* points[], int numPoints);

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

void initializeInsect()
{
    struct Point* middle[20];
    int midN = 0;
    int legsNum = 2;
    int p = 0;
    numSprings = 0;
    
    /* legs */
    for(int i = 0; i < legsNum; i ++)
    {
        initializePointsTetrahedral(p, i, 0.0, 2.0 / sqrt(3), -1.0, -1.0, 1.0);
        middle[midN++] = &points[p];
        middle[midN++] = &points[p + 1];
        numSprings += initializeSprings(numSprings, p, p + 4);
        p += 3;
    }
    middle[midN++] = &points[p];
    p ++;
    
    for(int i = 0; i < legsNum; i ++)
    {
        initializePointsTetrahedral(p, i, 1, 2.0 / sqrt(3), 1.0, -1.0, 1.0);
        middle[midN++] = &points[p];
        middle[midN++] = &points[p + 1];
        numSprings += initializeSprings(numSprings, p, p + 4);
        p += 3;
    }
    middle[midN++] = &points[p];
    p ++;
    
    /* body */
    int curMidN = midN;
    for(int i = 0; i < curMidN; i ++)
    {
        points[p].pos[0] = middle[i]->pos[0];
        points[p].pos[1] = middle[i]->pos[1];
        points[p].pos[2] = middle[i]->pos[2] + 0.5;
        middle[midN++] = &points[p];
        p ++;
    }

    initializeSpringsForEachCube(numSprings, middle, midN);
    
    numPoints = p;
    initializeAllPoints();
    
    return;
}

void initializeAllPoints()
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
    initializePointsTetrahedral(0, 0, 0, 0, 1, 1, 1);
}

void initializePointsTetrahedral(int pIdx, double leftX, double leftY, double leftZ, double yDir, double zDir, double sideLen)
{
    for(int i = 0; i < 4; i ++)
    {
        points[pIdx + i].mass = 0.1;
        for(int j = 0; j < 3; j ++)
        {
            points[pIdx + i].velocity[j] = 0;
            points[pIdx + i].accel[j] = 0;
            points[pIdx + i].force[j] = 0;
        }
    }
    
    double h = sqrt(3) * 0.5;
    double hmid = h / 3;
    double tetrahedralH = 2.0 / sqrt(3);
    double x[4] = {0, 0.5, 0.5, 1};
    double y[4] = {0, h, hmid, 0};
    double z[4] = {0, 0, tetrahedralH, 0};
    
    double drop_height = 0;
    
    int p = pIdx;
    for(int i = 0; i < 4; i ++)
    {
        points[p].pos[0] = (leftX + x[i]) * sideLen;
        points[p].pos[1] = (leftY + y[i] * yDir) * sideLen;
        points[p].pos[2] = (leftZ + z[i] * zDir)* sideLen + drop_height;
        p ++;
    }
}

int initializeSprings(int pIdx, int l, int r)
{
    int p = pIdx;
    for(int i = l; i < r - 1; i ++)
    {
        for(int j = i + 1; j < r; j ++)
        {
            springs[p].muscle = false;
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
    return p - pIdx;
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
    double y[5] = {0, 1, 2, 3};
    int p = 0;
    
    for(int i = 0; i < 4; i ++)
    {
        for(int j = 0; j < 4; j ++)
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
    double y[5] = {0, 1, 2, 3};
    int p = 0;
    
    for(int i = 0; i < 5; i ++)
    {
        for(int j = 0; j < 4; j ++)
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
    numPoints = 0;
    int p = 0;
    double dropHeight = 1.5;
    
    for(int z = 0; z <= 3; z ++)
    {
        if(z == 0 || z == 1)
        {
            p += initializeFeet(&points[p], z + dropHeight);
        }
        else
        {
            p += initializeBody(&points[p], z + dropHeight);
        }
    }
    numPoints = p;
    
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

void initializeSpringsForEachCube(int pIdx, struct Point* points[], int numPoints)
{
    int p = pIdx;
    
    for(int i = 0; i < numPoints - 1; i ++)
    {
        for(int j = i + 1; j < numPoints; j ++)
        {
            if(calcDist(points[i]->pos, points[j]->pos) > sqrt(3) + 0.01)
            {
                continue;
            }
            
            springs[p].p1 = points[i];
            springs[p].p2 = points[j];
            springs[p].len = calcDist(points[i]->pos, points[j]->pos);
            springs[p].k = 10000;
            p ++;
        }
    }
    
    numSprings = p;
    
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
