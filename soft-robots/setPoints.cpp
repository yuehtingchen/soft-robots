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

extern const float TIME_STEP;
const float GRAVITY[3] = {0, 0, -9.81};
const int kc = 10000; // restoration force
const bool damping = false;
const float DAMPING_CONST = 0.99999;
const bool breathing = false;
const float OMEGA = 3.1415926;
const float b = 0.05;

float T = 0;
int numPoints = 8;
int numSprings = numPoints * (numPoints - 1) / 2;
struct Point points[MAXN];
struct Spring springs[MAXN];
float energy[1000000][2];
int energy_len = 0;

void writeEnergy();
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
void initializePointsTetrahedral();
void initializeSprings();
float calcPotentialEnergy();
float calcKineticEnergy();

void updatePoints()
{
    calcForce();

//    float ke = calcKineticEnergy();
//    float pe = calcPotentialEnergy();
//    energy[energy_len][0] = ke;
//    energy[energy_len ++][1] = pe;

    updatePointsPos();
    T += TIME_STEP;
    
    return;
}

void writeEnergy()
{
    char filename[100] = "/Users/CJChen/Desktop/CourseworksF2022/energy/energy_spin.csv";
    FILE* file = fopen(filename, "w+");
    if(file == NULL)
    {
        perror(filename);
        exit(1);
    }
    
    for(int i = 0; i < energy_len; i ++)
    {
        fprintf(file, "%f, %f, %f\n", energy[i][0], energy[i][1], energy[i][0] + energy[i][1]);
    }
    
    fclose(file);
}

void printPoints()
{
    for(int i = 0; i < numPoints; i ++)
    {
        printf("%f %f %f\n", points[i].pos[0], points[i].pos[1], points[i].pos[2]);
        printf("F: %f %f %f\n", points[i].force[0], points[i].force[1], points[i].force[2]);
        printf("V: %f %f %f\n", points[i].velocity[0], points[i].velocity[1], points[i].velocity[2]);
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
            if(damping)
            {
                points[i].velocity[j] *= DAMPING_CONST;
            }
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
        
        if(breathing)
        {
            len = len + b * sin(OMEGA * T - TIME_STEP);
        }
        
        float dist = calcDist(p1->pos, p2->pos);
        float force = k * (dist - len);
        float vec_p1_p2[3];
        calcVector(vec_p1_p2, p1->pos, p2->pos);
        normalizeVector(vec_p1_p2);
        
        for(int j = 0; j < 3; j ++)
        {
            p1->force[j] += force * vec_p1_p2[j];
        }
        
        reverseVector(vec_p1_p2);
        
        for(int j = 0; j < 3; j ++)
        {
            p2->force[j] += force * vec_p1_p2[j];
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
    
    float h = sqrt(3) * 0.5;
    float hmid = h / 3;
    float x[4] = {0, 0.5, 1, 0.5};
    float y[4] = {0, h, 0, hmid};
    float z[4] = {0, 0, 0, h};
    
    float drop_height = 0.2;
    
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
            p ++;
        }
    }
}

float calcPotentialEnergy()
{
    float energy = 0;
    for(int i = 0; i < numPoints; i ++)
    {
        energy += points[i].mass * -1 * GRAVITY[2] * points[i].pos[2];
    }
    
    for(int i = 0; i < numSprings; i ++)
    {
        float len = springs[i].len;
        float k = springs[i].k;
        float* p1 = springs[i].p1->pos;
        float* p2 = springs[i].p2->pos;
        
        float x = calcDist(p1, p2) - len;
        
        energy += 0.5 * k * pow(x, 2);
    }
    
    for(int i = 0; i < numPoints; i ++)
    {
        if(points[i].pos[2] >= 0) continue;
        energy += 0.5 * kc * pow(points[i].pos[2], 2);
    }
    
    return energy;
}

float calcKineticEnergy()
{
    float energy = 0;
    for(int i = 0; i < numPoints; i ++)
    {
        float v = sqrt(pow(points[i].velocity[0], 2) + pow(points[i].velocity[1], 2) + pow(points[i].velocity[2], 2));
        energy += 0.5 * points[i].mass * pow(v, 2);
    }
    
    return energy;
}
