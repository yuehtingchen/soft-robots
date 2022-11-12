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
#include "createObject.hpp"
#include "utility.h"

extern const double TIME_STEP;
extern double T;

const double GRAVITY[3] = {0, 0, -9.81};
const int kc = 10000; // restoration force
const bool damping = true;
const double DAMPING_CONST = 0.9997;
const double mu = 1.0; // friction
const double muk = 0.8; // friction

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN_SQR];

double energy[1000000][2];
int energy_len = 0;

void writeEnergy();
void printSprings();
void printPoints();
void calcForce();
void updatePointsPos();
void resetPointForce();
void calcSpringForce();
void calcGravitationalForce();
void calcFriction();
void calcRestorationForce();
void calcVector(double vec[3], double p1[3], double p2[3]);
void normalizeVector(double vec[3]);
void reverseVector(double vec[3]);
double calcPotentialEnergy();
double calcKineticEnergy();


void updatePoints()
{
    calcForce();

//    double ke = calcKineticEnergy();
//    double pe = calcPotentialEnergy();
//    energy[energy_len][0] = ke;
//    energy[energy_len ++][1] = pe;

    updatePointsPos();
    return;
}

void writeEnergy()
{
    char filename[100] = "/Users/CJChen/Desktop/CourseworksF2022/energy/energy_friction.csv";
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

void printSprings()
{
    for(int i = 0; i < numSprings; i ++)
    {
        printf("p1: %f %f %f\n", springs[i].p1->pos[0], springs[i].p1->pos[1], springs[i].p1->pos[2]);
        printf("p2: %f %f %f\n", springs[i].p2->pos[0], springs[i].p2->pos[1], springs[i].p2->pos[2]);
        printf("k=%f ", springs[i].k);
        if(springs[i].muscle)
        {
            printf("omega=%f b=%f c=%f", springs[i].omega, springs[i].b, springs[i].c);
        }
        printf("\n");
    }
    printf("--------------------\n");
}

void calcForce()
{
    resetPointForce();
    calcSpringForce();
    calcGravitationalForce();
    calcFriction();
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
        double k = springs[i].k;
        double len = springs[i].len;
        
        if(springs[i].muscle)
        {
            len = len + springs[i].b * sin(springs[i].omega * T + springs[i].c);
        }
        
        double dist = calcDist(p1->pos, p2->pos);
        double force = k * (dist - len);
        double vec_p1_p2[3];
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

void calcFriction()
{
    for(int i = 0; i < numPoints; i ++)
    {
        if(points[i].pos[2] > 0 || points[i].force[2] > 0) continue;
        
        double Fp = sqrt(pow(points[i].force[0], 2) + pow(points[i].force[1], 2));
        double Fn = -1 * points[i].force[2];
        
        if(Fp < Fn * mu)
        {
            points[i].force[0] = points[i].force[0] - points[i].force[0];
            points[i].force[1] = points[i].force[1] - points[i].force[1];
        }
        else
        {
            double kF = Fn * muk;
            double a[3] = {points[i].force[0] / points[i].mass, points[i].force[1] / points[i].mass, 0.0};
            double v[3] = {points[i].velocity[0] + a[0] * TIME_STEP, points[i].velocity[1] + a[1] * TIME_STEP, 0};
            normalizeVector(v);

            for(int j = 0; j < 3; j ++)
            {
                points[i].force[j] = points[i].force[j] - v[j] * kF;
            }
        }
        
    }
}

void calcRestorationForce()
{
    for(int i = 0; i < numPoints; i ++)
    {
        if(points[i].pos[2] >= 0) continue;
        
        double force[3] = {0, 0, -1 * kc * points[i].pos[2]};
        for(int j = 0; j < 3; j ++)
        {
            points[i].force[j] += force[j];
        }
    }
    
    return;
}

void calcVector(double vec[3], double p1[3], double p2[3])
{
    vec[0] = p2[0] - p1[0];
    vec[1] = p2[1] - p1[1];
    vec[2] = p2[2] - p1[2];
    
    return;
}

void normalizeVector(double vec[3])
{
    double denom = sqrt(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2));
    for(int i = 0; i < 3; i ++)
    {
        vec[i] = vec[i] / denom;
    }
    
    return;
}

void reverseVector(double vec[3])
{
    for(int i = 0; i < 3; i ++)
    {
        vec[i] = vec[i] * -1;
    }
    
    return;
}

double calcPotentialEnergy()
{
    double energy = 0;
    for(int i = 0; i < numPoints; i ++)
    {
        energy += points[i].mass * -1 * GRAVITY[2] * points[i].pos[2];
    }
    
    for(int i = 0; i < numSprings; i ++)
    {
        double len = springs[i].len;
        double k = springs[i].k;
        double* p1 = springs[i].p1->pos;
        double* p2 = springs[i].p2->pos;
        
        double x = calcDist(p1, p2) - len;
        
        energy += 0.5 * k * pow(x, 2);
    }
    
    for(int i = 0; i < numPoints; i ++)
    {
        if(points[i].pos[2] >= 0) continue;
        energy += 0.5 * kc * pow(points[i].pos[2], 2);
    }
    
    return energy;
}

double calcKineticEnergy()
{
    double energy = 0;
    for(int i = 0; i < numPoints; i ++)
    {
        double v = sqrt(pow(points[i].velocity[0], 2) + pow(points[i].velocity[1], 2) + pow(points[i].velocity[2], 2));
        energy += 0.5 * points[i].mass * pow(v, 2);
    }
    
    return energy;
}
