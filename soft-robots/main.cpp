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
// #include "draw.hpp"

extern const double TIME_STEP = 0.0001;
extern const double MAX_TIME = 15.0;
double T = 0;

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN];

const int testNum = 5;
const int evaluationTimes = 10;
const int sampleSize = 10;
const int selectInterval = 5;

double bestSpeed[evaluationTimes];
char folderName[100] = "/home/yc3877/soft-robots-data/";
char filenameSpeed[100];
char filenameMaterial[100];

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
    selectRun = 2;
    /* 0: cube, 1: 2 cubes, 2: walking cubes*/
    selectObject = 2;
    
    strcat(folderName, "walking-cubes/");

    auto start = chrono::high_resolution_clock::now();
    for(int i = 0; i < testNum; i ++)
    {
        filenameSpeed[0] = 0;
        filenameMaterial[0] = 0;
        strcat(filenameSpeed, folderName);
        strcat(filenameMaterial, folderName);

        char tmpfilenameSpeed[100];
        char tmpfilenameMaterial[100];
        snprintf(tmpfilenameSpeed, 12, "speed_%d.csv", i + 1);
        snprintf(tmpfilenameMaterial, 15, "material_%d.txt", i + 1);

        if(selectRun == 0)
        {
            strcat(filenameSpeed, "RM/");
            strcat(filenameSpeed, tmpfilenameSpeed);

            strcat(filenameMaterial, "RM/");
            strcat(filenameMaterial, tmpfilenameMaterial);
            randomSearch();
        }
        else if(selectRun == 1)
        {
            strcat(filenameSpeed, "HC/");
            strcat(filenameSpeed, tmpfilenameSpeed);

            strcat(filenameMaterial, "HC/");
            strcat(filenameMaterial, tmpfilenameMaterial);
            hillClimber();
        }
        else if(selectRun == 2)
        {
            strcat(filenameSpeed, "EA/");
            strcat(filenameSpeed, tmpfilenameSpeed);

            strcat(filenameMaterial, "EA/");
            strcat(filenameMaterial, tmpfilenameMaterial);
            evolutionAlgo();
        }
        writeSpeed();
    }
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "Time taken by function: "
         << duration.count() / 1000000 << " seconds" << endl;
    
    /* draw best robot */
    /*
    initObject();
    strcat(filenameMaterial, folderName);
    strcat(filenameMaterial, "EA/material_1.txt");
    struct Material materials[MAXN];
    int materialsNum = 0;
    materialsNum = readMaterial(filenameMaterial, materials);
    applyMaterialtoSprings(materials, materialsNum);
    draw();
    */ 
    return 0;
}


void randomSearch()
{
    double individualSpeed[sampleSize];
    int individualMaterialNum[sampleSize];
    struct Material individualMaterial[sampleSize][MAXN];

    for(int sample = 0; sample < sampleSize; sample ++)
    {
        initObject();
        randInitMaterial(individualMaterial[sample], &individualMaterialNum[sample]);
        applyMaterialtoSprings(individualMaterial[sample], individualMaterialNum[sample]);
        individualSpeed[sample] = speed(points);
    }
    
    int bestMaterialIdx = 0;
    double maxSpeed = 0;
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        for(int sample = 0; sample < sampleSize; sample ++)
        {
            initObject();
            randInitMaterial(individualMaterial[sample], &individualMaterialNum[sample]);
            applyMaterialtoSprings(individualMaterial[sample], individualMaterialNum[sample]);
            individualSpeed[sample] = speed(points);
            
            if(individualSpeed[sample] > maxSpeed)
            {
                maxSpeed = individualSpeed[sample];
                bestMaterialIdx = sample;
            }
        }
        
        bestSpeed[i] = maxSpeed;
    }
    
    applyMaterialtoSprings(individualMaterial[bestMaterialIdx], individualMaterialNum[bestMaterialIdx]);
    printf("speed: %f\n", maxSpeed);
    
    FILE* fileMaterial = fopen(filenameMaterial, "w+");
    if(fileMaterial == NULL)
    {
        perror(filenameSpeed);
        exit(1);
    }
    writeMaterial(fileMaterial, individualMaterial[bestMaterialIdx], individualMaterialNum[bestMaterialIdx]);
    fclose(fileMaterial);
    
    return;
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
    double individualSpeed[sampleSize];
    int individualMaterialNum[sampleSize];
    struct Material individualMaterial[sampleSize][MAXN];

    for(int sample = 0; sample < sampleSize; sample ++)
    {
        initObject();
        randInitMaterial(individualMaterial[sample], &individualMaterialNum[sample]);
        applyMaterialtoSprings(individualMaterial[sample], individualMaterialNum[sample]);
        individualSpeed[sample] = speed(points);
    }
    
    int bestMaterialIdx = 0;
    double maxSpeed = 0;
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        bestMaterialIdx = 0;
        maxSpeed = 0;
        for(int sample = 0; sample < sampleSize; sample ++)
        {
            hillClimbStep(individualMaterial[sample], individualMaterialNum[sample], individualSpeed[sample]);
            
            if(individualSpeed[sample] > maxSpeed)
            {
                maxSpeed = individualSpeed[sample];
                bestMaterialIdx = sample;
            }
        }
        
        bestSpeed[i] = maxSpeed;
    }
    
    applyMaterialtoSprings(individualMaterial[bestMaterialIdx], individualMaterialNum[bestMaterialIdx]);
    printf("speed: %f\n", maxSpeed);
    
    FILE* fileMaterial = fopen(filenameMaterial, "w+");
    if(fileMaterial == NULL)
    {
        perror(filenameSpeed);
        exit(1);
    }
    writeMaterial(fileMaterial, individualMaterial[bestMaterialIdx], individualMaterialNum[bestMaterialIdx]);
    fclose(fileMaterial);
    
    return;
}

void updateIndividualSpeed(
    struct Material individualMaterial[sampleSize][MAXN],
    int individualMaterialNum[sampleSize],
    double individualSpeed[sampleSize])
{
    for(int i = 0; i < sampleSize; i ++)
    {
        initObject();
        applyMaterialtoSprings(individualMaterial[i], individualMaterialNum[i]);
        individualSpeed[i] = speed(points);
    }
    
    return;
}

void evolutionAlgo()
{
    double individualSpeed[sampleSize];
    int individualMaterialNum[sampleSize];
    struct Material individualMaterial[sampleSize][MAXN];

    for(int sample = 0; sample < sampleSize; sample ++)
    {
        initObject();
        randInitMaterial(individualMaterial[sample], &individualMaterialNum[sample]);
        applyMaterialtoSprings(individualMaterial[sample], individualMaterialNum[sample]);
        individualSpeed[sample] = speed(points);
    }
    
    int bestMaterialIdx = 0;
    double maxSpeed = 0;
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        bestMaterialIdx = 0;
        maxSpeed = 0;
        for(int sample = 0; sample < sampleSize; sample ++)
        {
            hillClimbStep(individualMaterial[sample], individualMaterialNum[sample], individualSpeed[sample]);
            
            if(individualSpeed[sample] > maxSpeed)
            {
                maxSpeed = individualSpeed[sample];
                bestMaterialIdx = sample;
            }
        }
        bestSpeed[i] = maxSpeed;
        
        if((i + 1) % selectInterval == 0 && (i + 1) != evaluationTimes)
        {
            basicSelect(individualMaterial, individualMaterialNum, individualSpeed);
            updateIndividualSpeed(individualMaterial, individualMaterialNum, individualSpeed);
        }
    }
    
    applyMaterialtoSprings(individualMaterial[bestMaterialIdx], individualMaterialNum[bestMaterialIdx]);
    printf("speed: %f\n", maxSpeed);
    
    FILE* fileMaterial = fopen(filenameMaterial, "w+");
    if(fileMaterial == NULL)
    {
        perror(filenameSpeed);
        exit(1);
    }
    writeMaterial(fileMaterial, individualMaterial[bestMaterialIdx], individualMaterialNum[bestMaterialIdx]);
    fclose(fileMaterial);
    
    return;
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
    printf("\n");
    
    return;
}
