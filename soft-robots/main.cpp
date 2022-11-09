//
//  main.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/10/21.
//

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "utility.h"
#include "setPoints.hpp"
#include "createObject.hpp"
#include "evolve.hpp"
#include "draw.hpp"

extern const double TIME_STEP = 0.00005;
extern const double MAX_TIME = 15.0;
double T = 0;

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN];

const int testNum = 5;
const int evaluationTimes = 100;
const int sampleSize = 10;

void randomSearch();
void hillClimber();
void evolutionAlgo();

void printMaterials(struct Material materials[MAXN], int materialsNum)
{
    for(int i = 0; i < materialsNum; i ++)
    {
        printf("location=(%f %f %f)\n", materials[i].p->pos[0], materials[i].p->pos[1], materials[i].p->pos[2]);
        printf("muscle = %d\n", materials[i].muscle);
        if(materials[i].muscle)
        {
            printf("b = %f\n", materials[i].b);
        }
        else
        {
            printf("k = %f\n", materials[i].k);
        }
    }
}

int main()
{
    srand(time(NULL));
    
    struct Material materials[MAXN];
    int materialsNum = 0;
    initializeWalkingCubes();
    randInitMaterial(materials, &materialsNum);
    initSpringsMaterial(materials, materialsNum);
    printMaterials(materials, materialsNum);
    
    draw();

//    auto start = chrono::high_resolution_clock::now();
//    while(T < MAX_TIME)
//    {
//        updatePoints();
//    }
//
//    auto stop = chrono::high_resolution_clock::now();
//    auto duration = duration_cast<chrono::microseconds>(stop - start);
//    cout << "Time taken by function: "
//         << duration.count() << " microseconds" << endl;
//    writeEnergy();
    
    return 0;
}

void randomSearch()
{
    double maxSpeed = 0;
    int bestMaterialNum = 0;
    struct Material bestMaterial[MAXN];
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        printf("evaluating %d\n", i);
        struct Material materials[MAXN];
        int materialsNum = 0;
        
        initializeWalkingCubes();
        randInitMaterial(materials, &materialsNum);
        initSpringsMaterial(materials, materialsNum);
        
        double sp = speed(points);
        
        if(sp > maxSpeed)
        {
            maxSpeed = sp;
            for(int i = 0; i < materialsNum; i ++)
            {
                bestMaterial[i] = materials[i];
            }
            bestMaterialNum = materialsNum;
        }
    }
    
    initSpringsMaterial(bestMaterial, bestMaterialNum);
    printf("speed: %f\n", maxSpeed);
}
