//
//  main.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/10/21.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <math.h>
using namespace std;

#include "utility.h"
#include "setPoints.hpp"
#include "createObject.hpp"
#include "evolve.hpp"
#include "draw.hpp"

extern const double TIME_STEP = 0.0001;
extern const double MAX_TIME = 15.0;
double T = 0;

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN];

const int testNum = 1;
const int evaluationTimes = 10;
const int sampleSize = 10;

double bestSpeed[evaluationTimes];
char filenameSpeed[100] = "/Users/CJChen/Desktop/CourseworksF2022/softRobotDocs/data/";
char filenameMaterial[100] = "/Users/CJChen/Desktop/CourseworksF2022/softRobotDocs/data/";

int selectRun = 0;
int selectObject = 0;

void randomSearch();
void hillClimber();
void hillClimbStep(struct Material bestMaterial[MAXN], int* bestMaterialNum, int* maxSpeed);
void evolutionAlgo();

/* helper functions */
void initObject();
void writeSpeed();
void writeMaterial(FILE* file, struct Material materials[MAXN], int materialsNum);
int readMaterial(char filename[100], struct Material materials[MAXN]);
void printMaterials(struct Material materials[MAXN], int materialsNum);

int main()
{
    srand((unsigned int)time(NULL));
    
    /* 0: random, 1: hillClimber, 2: evolutionAlgo */
    selectRun = 1;
    /* 0: cube, 1: 2 cubes, 2: walking cubes*/
    selectObject = 1;

    strcat(filenameSpeed, "twoCubes/");
    strcat(filenameMaterial, "twoCubes/");

    auto start = chrono::high_resolution_clock::now();
    for(int i = 0; i < testNum; i ++)
    {
        if(selectRun == 0)
        {
            strcat(filenameSpeed, "RM/speed.csv");
            strcat(filenameMaterial, "RM/material.txt");
            randomSearch();
        }
        else if(selectRun == 1)
        {
            strcat(filenameSpeed, "HC/speed.csv");
            strcat(filenameMaterial, "HC/material.txt");
            hillClimber();
        }
        else if(selectRun == 2)
        {
            strcat(filenameSpeed, "EA/speed.csv");
            strcat(filenameMaterial, "EA/material.txt");
            hillClimber();
        }
        writeSpeed();
    }
    auto stop = chrono::high_resolution_clock::now();
    auto duration = duration_cast<chrono::microseconds>(stop - start);
    cout << "Time taken by function: "
         << duration.count() << " microseconds" << endl;
    
    /* draw best robot */
    initObject();
    struct Material materials[MAXN];
    int materialsNum = 0;
    materialsNum = readMaterial(filenameMaterial, materials);
    applyMaterialtoSprings(materials, materialsNum);
    draw();
    
    return 0;
}


void randomSearch()
{
    FILE* fileMaterial = fopen(filenameMaterial, "w+");
    if(fileMaterial == NULL)
    {
        perror(filenameSpeed);
        exit(1);
    }

    double maxSpeed = 0;
    int bestMaterialNum = 0;
    struct Material bestMaterial[MAXN];
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        struct Material materials[MAXN];
        int materialsNum = 0;
        
        initObject();
        randInitMaterial(materials, &materialsNum);
        applyMaterialtoSprings(materials, materialsNum);
        
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
        
        bestSpeed[i] = maxSpeed;
    }
    
    applyMaterialtoSprings(bestMaterial, bestMaterialNum);
    printf("speed: %f\n", maxSpeed);
    
    printMaterials(bestMaterial, bestMaterialNum);
    
    writeMaterial(fileMaterial, bestMaterial, bestMaterialNum);
    fclose(fileMaterial);
}

static void hillClimbStep(struct Material *bestMaterial, int &bestMaterialNum, double &maxSpeed) {
    struct Material materials[MAXN];
    int materialsNum = bestMaterialNum;
    
    for(int i = 0; i < bestMaterialNum; i ++)
    {
        materials[i] = bestMaterial[i];
    }
    
    initObject();
    mutateMaterial(materials, &materialsNum);
    applyMaterialtoSprings(materials, materialsNum);
    
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
    
    return;
}

void hillClimber()
{
    FILE* fileMaterial = fopen(filenameMaterial, "w+");
    if(fileMaterial == NULL)
    {
        perror(filenameSpeed);
        exit(1);
    }
    
    double maxSpeed = 0;
    int bestMaterialNum = 0;
    struct Material bestMaterial[MAXN];
    initObject();
    randInitMaterial(bestMaterial, &bestMaterialNum);
    applyMaterialtoSprings(bestMaterial, bestMaterialNum);
    maxSpeed = speed(points);
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        hillClimbStep(bestMaterial, bestMaterialNum, maxSpeed);
        bestSpeed[i] = maxSpeed;
    }
    
    applyMaterialtoSprings(bestMaterial, bestMaterialNum);
    printf("speed: %f\n", maxSpeed);
    
    writeMaterial(fileMaterial, bestMaterial, bestMaterialNum);
    fclose(fileMaterial);
}

void evolutionAlgo()
{
    double maxSpeed[sampleSize];
    int bestMaterialNum[sampleSize];
    struct Material bestMaterial[sampleSize][MAXN];
    
    for(int sample = 0; sample < sampleSize; sample ++)
    {
        initObject();
        randInitMaterial(bestMaterial[sample], &bestMaterialNum[sample]);
        applyMaterialtoSprings(bestMaterial[sample], bestMaterialNum[sample]);
        maxSpeed[sample] = speed(points);
    }
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        for(int sample = 0; sample < sampleSize; sample ++)
        {
            initObject();
            randInitMaterial(bestMaterial[sample], &bestMaterialNum[sample]);
            applyMaterialtoSprings(bestMaterial[sample], bestMaterialNum[sample]);
            maxSpeed[sample] = speed(points);
        }
    }
}

/* helper methods */
void initObject()
{
    if(selectObject == 0)
    {
        initializeCube();
    }
    else if(selectObject == 1)
    {
        initializeTwoCubes();
    }
    else if(selectObject == 2)
    {
        initializeWalkingCubes();
    }
    else
    {
        initializeCube();
    }
    
    return;
}

void writeSpeed()
{
    FILE* file = fopen(filenameSpeed, "w+");
    if(file == NULL)
    {
        perror(filenameSpeed);
        exit(1);
    }
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        fprintf(file, "%lf\n", bestSpeed[i]);
    }
    
    fclose(file);
    
    return;
}

void writeMaterial(FILE* file, struct Material materials[MAXN], int materialsNum)
{
    /* int Idx;
     * double len;
     * double k;
     * bool muscle = false;
     * double omega;
     * double b;
     * double c;
     */
    for(int i = 0; i < materialsNum; i ++)
    {
        fprintf(file, "material\n");
        fprintf(file, "%d\n", materials[i].pIdx);
        fprintf(file, "%lf\n", materials[i].len);
        fprintf(file, "%lf\n", materials[i].k);
        fprintf(file, "%d\n", materials[i].muscle);
        fprintf(file, "%lf\n", materials[i].omega);
        fprintf(file, "%lf\n", materials[i].b);
        fprintf(file, "%lf\n", materials[i].c);
        fprintf(file, "\n");
    }
    fprintf(file, "\n");
    
    return;
}

int readMaterial(char filename[100], struct Material materials[MAXN])
{
    FILE* file = fopen(filenameMaterial, "r");
    if(file == NULL)
    {
        perror(filenameSpeed);
        exit(1);
    }
    
    int materialsNum = 0;
    while(fscanf(file, "material\n") != EOF)
    {
        fscanf(file, "%d\n", &materials[materialsNum].pIdx);
        fscanf(file, "%lf\n", &materials[materialsNum].len);
        fscanf(file, "%lf\n", &materials[materialsNum].k);
        fscanf(file, "%d\n", &materials[materialsNum].muscle);
        fscanf(file, "%lf\n", &materials[materialsNum].omega);
        fscanf(file, "%lf\n", &materials[materialsNum].b);
        fscanf(file, "%lf\n", &materials[materialsNum].c);
        fscanf(file, "\n");
        
        materialsNum ++;
    }
    
    fclose(file);
    return materialsNum;
}

void printMaterials(struct Material materials[MAXN], int materialsNum)
{
    for(int i = 0; i < materialsNum; i ++)
    {
        int idx = materials[i].pIdx;
        printf("location=(%f %f %f)\n", points[idx].pos[0], points[idx].pos[1], points[idx].pos[2]);
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
    
    return;
}
