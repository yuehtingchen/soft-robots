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

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN];

const int K_HARD = 10000;
const int K_SOFT = 100;
const float PI = 3.1415926;

int materialsNum = 0;
struct Material materials[MAXN];

void initSpringsMaterial();
void randMaterial();
void hardSupport(struct Material* material);
void softSupport(struct Material* material);
void randMuscle(struct Material* material);
int random(int low, int high);

void initSpringsMaterial()
{
    for(int i = 0; i < numSprings; i ++)
    {
        /* find closest material */
        struct Spring *spring = &springs[i];
        
        int materialIdx = 0;
        float minDist = calcDist(spring->p1->pos, materials[0].p->pos);
        for(int j = 1; j < materialsNum; j ++)
        {
            float dist = calcDist(spring->p1->pos, materials[j].p->pos);
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
 * hard support (0),
 * soft support (1), or
 * random muscle (2, 3) (either contract or expand first)
 *
 */
void randMaterial()
{
    materialsNum = 4;
    
    for(int i = 0; i < materialsNum; i ++)
    {
        int selectMaterial = random(0, 3);
        int selectLocation = random(numPoints / materialsNum * i, numPoints / materialsNum * (i + 1));
        
        /* hard support */
        if(selectMaterial == 0)
        {
            hardSupport(&materials[i]);
            materials[i].p = &points[selectLocation];
        }
        /* soft support */
        else if(selectMaterial == 1)
        {
            softSupport(&materials[i]);
            materials[i].p = &points[selectLocation];
        }
        /* random muscle */
        else if(selectMaterial == 2 || selectMaterial == 3)
        {
            randMuscle(&materials[i]);
            materials[i].p = &points[selectLocation];
        }
    }
}

/*
 * Possible mutations include
 * changing
 */
void mutateMaterial()
{
    
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
    material->b = random(1, 10) * 0.1;
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
        material->c = PI / 2 * -1;
    }
}

int random(int low, int high)
{
    return rand() % (high + 1 - low) + low;
}

