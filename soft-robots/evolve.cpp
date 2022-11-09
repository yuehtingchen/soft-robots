//
//  evolve.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/7.
//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "evolve.hpp"
#include "utility.h"
#include "createObject.hpp"
#include "setPoints.hpp"

extern double T;
extern const double TIME_STEP;
extern const double MAX_TIME;

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN];

const int K_HARD = 10000;
const int K_SOFT = 1000;
const double PI = 3.1415926;

int materialsNum = 0;
struct Material materials[MAXN];

void initSpringsMaterial(struct Material materials[MAXN], int materialsNum);
void randInitMaterial(struct Material materials[MAXN], int* materialsNum);
void randMaterial(struct Material* material, struct Point* location);
void hardSupport(struct Material* material);
void softSupport(struct Material* material);
void randMuscle(struct Material* material);
void getCenterOfMass(struct Point points[MAXN], double centerPos[2]);
int random(int low, int high);

double speed(struct Point points[MAXN])
{
    double initXY[3];
    getCenterOfMass(points, initXY);
    
    T = 0;
    while(T <= MAX_TIME)
    {
        updatePoints();
        T += TIME_STEP;
    }
    
    double finalXY[3];
    getCenterOfMass(points, finalXY);
    
    return calcDist(initXY, finalXY) / (double) MAX_TIME;
}

void initSpringsMaterial(struct Material materials[MAXN], int materialsNum)
{
    for(int i = 0; i < numSprings; i ++)
    {
        /* find closest material */
        struct Spring *spring = &springs[i];
        
        int materialIdx = 0;
        double minDist = calcDist(spring->p1->pos, materials[0].p->pos);
        for(int j = 1; j < materialsNum; j ++)
        {
            double dist = calcDist(spring->p1->pos, materials[j].p->pos);
            if(dist < minDist)
            {
                materialIdx = j;
            }
        }
        
        /* apply material */
        struct Material material = materials[materialIdx];
        spring->k = material.k;
        spring->muscle = material.muscle;
        spring->omega = material.omega;
        spring->b = material.b;
        spring->c = material.c;
    }
}

/*
 * randomly choose 4 materials
 */
void randInitMaterial(struct Material materials[MAXN], int* materialsNum)
{
    *materialsNum = 4;
    
    for(int i = 0; i < *materialsNum; i ++)
    {
        int selectLocation = random(numPoints / *materialsNum * i, numPoints / *materialsNum * (i + 1));
        
        randMaterial(&materials[i], &points[selectLocation]);
    }
}

/*
 * Possible mutations include
 * 1. replacing one type of material to another
 * 2. swapping location of two materials
 * 3. remove material
 * 4. add material
 */
void mutateMaterial(struct Material materials[MAXN], int* materialsNum)
{
    struct Material newMaterials[MAXN];
    int newMaterialsNum = *materialsNum;
    for(int i = 0; i < *materialsNum; i ++)
    {
        newMaterials[i] = materials[i];
    }
    
    int selectMutate = random(1, 4);
    if(selectMutate == 1)
    {
        int selectMaterialIdx = random(0, *materialsNum - 1);
        struct Material *material = &newMaterials[selectMaterialIdx];
        randMaterial(material, material->p);
    }
    else if(selectMutate == 2)
    {
        int selectMaterialIdx1 = random(0, *materialsNum - 1);
        int selectMaterialIdx2 = (selectMaterialIdx1 + random(1, *materialsNum - 1)) % *materialsNum; // prevent choosing the same index
        
        newMaterials[selectMaterialIdx1].p = materials[selectMaterialIdx2].p;
        newMaterials[selectMaterialIdx2].p = materials[selectMaterialIdx1].p;
    }
    else if(selectMutate == 3)
    {
        int selectMaterialIdx = random(0, *materialsNum - 1);
        
        int p = 0;
        for(int i = 0; i < *materialsNum; i ++)
        {
            if(i == selectMaterialIdx) continue;
            newMaterials[p ++] = materials[i];
        }
        
        newMaterialsNum = p;
    }
    else
    {
        /* check if two material have the same location */
        int selectLocation = 0;
        bool invalid = true;
        
        while(invalid)
        {
            selectLocation = random(0, numPoints - 1);
            invalid = false;
            
            for(int i = 0; i < *materialsNum; i ++)
            {
                if(materials[i].p == &points[selectLocation])
                {
                    invalid = true;
                }
            }
        }
        
        randMaterial(&newMaterials[*materialsNum], &points[selectLocation]);
        newMaterialsNum ++;
    }
    
    for(int i = 0; i < *materialsNum + 1; i ++)
    {
        materials[i] = newMaterials[i];
    }
    *materialsNum = newMaterialsNum;
    
    return;
}

void crossOver(struct Material materials1[MAXN], int* materialsNum1, struct Material materials2[MAXN], int* materialsNum2)
{
    
}

/*
 * hard support (0),
 * soft support (1), or
 * random muscle (2, 3) (either contract or expand first)
 *
 */
void randMaterial(struct Material* material, struct Point* location)
{
    int selectMaterial = random(0, 3);
    
    /* hard support */
    if(selectMaterial == 0)
    {
        hardSupport(material);
        material->p = location;
    }
    /* soft support */
    else if(selectMaterial == 1)
    {
        softSupport(material);
        material->p = location;
    }
    /* random muscle */
    else if(selectMaterial == 2 || selectMaterial == 3)
    {
        randMuscle(material);
        material->p = location;
    }
}

/*
 * initialize hard support material
 */
void hardSupport(struct Material* material)
{
    material->muscle=false;
    material->k = K_HARD;
}

/*
 * initialize soft support material
 */

void softSupport(struct Material* material)
{
    material->muscle=false;
    material->k = K_SOFT;
}

/*
 * initialize random muscle
 * random: b, c
 */
void randMuscle(struct Material* material)
{
    material->muscle = true;
    material->b = random(3, 5) * 0.1;
    material->omega = PI;
    material->k = 1000;
    
    /* expand */
    if(random(0, 1) == 0)
    {
        material->c = 0;
    }
    /* contract */
    else
    {
        material->c = PI * -1;
    }
}

void getCenterOfMass(struct Point points[MAXN], double centerPos[3])
{
    double x = 0, y = 0;
    double total_mass = 0;
    
    for(int i = 0; i < numPoints; i ++)
    {
        x += points[i].mass * points[i].pos[0];
        y += points[i].mass * points[i].pos[1];
        total_mass += points[i].mass;
    }
    
    x /= total_mass;
    y /= total_mass;
    
    centerPos[0] = x;
    centerPos[1] = y;
    centerPos[2] = 0;
    
    return;
}

int random(int low, int high)
{
    return rand() % (high + 1 - low) + low;
}

