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
extern const int sampleSize = 10;

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN_SQR];

const int K_HARD = 10000;
const int K_SOFT = 1000;
const double PI = 3.1415926;

void applyMaterialtoSprings(struct Material materials[MAXN], int materialsNum);
void randMaterial(struct Material* material, int location);
void randMaterial(struct Material* material, struct Point* location);
void hardSupport(struct Material* material);
void softSupport(struct Material* material);
void randMuscle(struct Material* material);
void getCenterOfMass(struct Point points[MAXN], double centerPos[3]);
void copyMaterial(struct Material* materialSrc, struct Material* materialDest, int materialsNum);
int random(int low, int high);

void speed(struct Point points[MAXN], double& speed, double& speedPath)
{
    double initXY[3];
    getCenterOfMass(points, initXY);
    
    double pathLen = 0;
    double prevXY[3] = {initXY[0], initXY[1], initXY[2]};
    
    T = 0;
    while(T <= MAX_TIME)
    {
        updatePoints();
        T += TIME_STEP;
        
        double curXY[3];
        getCenterOfMass(points, curXY);
        pathLen += calcDist(prevXY, curXY);
        prevXY[0] = curXY[0];
        prevXY[1] = curXY[1];
        prevXY[2] = curXY[2];
    }
    
    double finalXY[3];
    getCenterOfMass(points, finalXY);
    
    speedPath = pathLen / (double) MAX_TIME;
    speed = calcDist(initXY, finalXY) / (double) MAX_TIME;
    
    return;
}

double speedFitness(double speed, double speedPath)
{
    return speed * 2 - speedPath;
}

void applyMaterialtoSprings(struct Material materials[MAXN], int materialsNum)
{
    for(int i = 0; i < numSprings; i ++)
    {
        /* find closest material */
        struct Spring *spring = &springs[i];
        
        int materialIdx = 0;
        double minDist = calcDist(spring->p1->pos, points[materials[0].pIdx].pos);
        for(int j = 1; j < materialsNum; j ++)
        {
            double dist = calcDist(spring->p1->pos, points[materials[j].pIdx].pos);
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
        randMaterial(&materials[i], selectLocation);
    }
}

/*
 * Possible mutations include
 * 1. replacing one type of material to another
 * 2. swapping location of two materials
 * 3. add material
 * 4. remove material
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
    if(selectMutate == 3 && *materialsNum <= 2)
    {
        selectMutate = random(1, 3);
    }
    
    if(selectMutate == 1)
    {
        int selectMaterialIdx = random(0, *materialsNum - 1);
        struct Material *material = &newMaterials[selectMaterialIdx];
        randMaterial(material, material->pIdx);
    }
    else if(selectMutate == 2)
    {
        int selectMaterialIdx1 = random(0, *materialsNum - 1);
        int selectMaterialIdx2 = (selectMaterialIdx1 + random(1, *materialsNum - 1)) % *materialsNum; // prevent choosing the same index
        
        newMaterials[selectMaterialIdx1].pIdx = materials[selectMaterialIdx2].pIdx;
        newMaterials[selectMaterialIdx2].pIdx = materials[selectMaterialIdx1].pIdx;
    }
    else if(selectMutate == 3)
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
              if(materials[i].pIdx == selectLocation)
              {
                  invalid = true;
              }
          }
      }
      
      randMaterial(&newMaterials[*materialsNum], selectLocation);
      newMaterialsNum ++;
    }
    else
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
    
    for(int i = 0; i < *materialsNum + 1; i ++)
    {
        materials[i] = newMaterials[i];
    }
    *materialsNum = newMaterialsNum;
    
    return;
}


/* overwrite from left index (inclusive) to right index (not inclusive) */
void crossOver(
    struct Material materials1[MAXN],
    int* materialsNum1,
    struct Material materials2[MAXN],
    int* materialsNum2,
    struct Material offspring[MAXN],
    int* offspringNum)
{
    *offspringNum = *materialsNum2;
    for(int i = 0; i < *materialsNum2; i ++)
    {
        offspring[i] = materials2[i];
    }
    
    int minMaterialsNum = *materialsNum1 < *materialsNum2 ? *materialsNum1 : *materialsNum2;
    int left = random(0, minMaterialsNum - 2);
    int right = random(left + 1, minMaterialsNum - 1);
    int offset1 = random(0, *materialsNum1 - minMaterialsNum);
    int offset2 = random(0, *materialsNum2 - minMaterialsNum);
    
    for(int i = left; i < right; i ++)
    {
        offspring[i + offset2] = materials1[i + offset1];
    }
    
    return;
}

void basicSelect(struct Material materials[sampleSize][MAXN], int materialsNum[sampleSize], double speed[sampleSize])
{
    double fitness[sampleSize];
    int pressure = 0.5 * sampleSize;
    double thresholdFitness = 0;
    
    for(int i = 0; i < sampleSize; i ++)
    {
        fitness[i] = speed[i];
    }
    
    for(int i = 0; i < sampleSize; i ++)
    {
        int cnt = 0;
        for(int j = 0; j < sampleSize; j ++)
        {
            if(fitness[j] > fitness[i]) cnt ++;
        }
        if(cnt == pressure)
        {
            thresholdFitness = fitness[i];
        }
    }
    
    struct Material oldMaterials[sampleSize][MAXN];
    int oldMaterialsNum[sampleSize];
    double oldSpeed[sampleSize];
    for(int i = 0; i < sampleSize; i ++)
    {
        copyMaterial(materials[i], oldMaterials[i], materialsNum[i]);
        oldMaterialsNum[i] = materialsNum[i];
        oldSpeed[i] = speed[i];
    }
    
    for(int i = 0; i < sampleSize; i ++)
    {
        if(speed[i] < thresholdFitness)
        {
            int momIdx = random(0, sampleSize - 2);
            int dadIdx = random(momIdx + 1, sampleSize - 1);
            crossOver(oldMaterials[momIdx], &oldMaterialsNum[momIdx],
                      oldMaterials[dadIdx], &oldMaterialsNum[dadIdx],
                      materials[i], &materialsNum[i]);
        }
    }
    
    return;
}

/*
 * hard support (0),
 * soft support (1), or
 * random muscle (2, 3) (either contract or expand first)
 *
 */
void randMaterial(struct Material* material, int location)
{
    int selectMaterial = random(0, 3);
    
    /* hard support */
    if(selectMaterial == 0)
    {
        hardSupport(material);
        material->pIdx = location;
    }
    /* soft support */
    else if(selectMaterial == 1)
    {
        softSupport(material);
        material->pIdx = location;
    }
    /* random muscle */
    else if(selectMaterial == 2 || selectMaterial == 3)
    {
        randMuscle(material);
        material->pIdx = location;
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
    material->omega = random(3, 5);
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

void copyMaterial(struct Material* materialSrc, struct Material* materialDest, int materialsNum)
{
    for(int i = 0; i < materialsNum; i ++)
    {
        materialDest[i] = materialSrc[i];
    }
}

int random(int low, int high)
{
    return rand() % (high + 1 - low) + low;
}

