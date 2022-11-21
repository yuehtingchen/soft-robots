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
#include <sys/types.h>
#include <unistd.h>
using namespace std;

#include "utility.h"
#include "setPoints.hpp"
#include "createObject.hpp"
#include "evolve.hpp"
#include "evolveBody.hpp"
#include "draw.hpp"

extern const double TIME_STEP = 0.0001;
extern const double MAX_TIME = 10.0;
double T = 0;

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN];

bool toWriteDiversity = true;

double bestSpeed[evaluationTimes];
char folderName[100] = "/Users/CJChen/Desktop/CourseworksF2022/softRobotDocs/data/";
char filenameSpeed[100];
char filenameMaterial[100];
char filenameRules[100];
char filenameDiversity[100];

int selectRun = 0;
int selectObject = 0;

void randomSearch();
void hillClimber();
static void hillClimbStep(struct Material *bestMaterial, int &bestMaterialNum, double &maxSpeed, double &maxSpeedPath, bool bestRules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void evolutionAlgo();

/* helper functions */
void initObject();
void initObject(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void writeSpeed();
void testOpenFile();
void writeDiversity(double diversity[evaluationTimes]);
void writeMaterial(FILE* file, struct Material materials[MAXN], int materialsNum);
int readMaterial(char filename[100], struct Material materials[MAXN]);
void writeRules(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void readRules(char filenameRules[100], bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void printMaterials(struct Material materials[MAXN], int materialsNum);

int main()
{
    /* 0: random, 1: hillClimber, 2: evolutionAlgo */
    selectRun = 2;
    /* 0: cube, 1: 2 cubes, 2: walking cubes, 3: insect*/
    selectObject = 0;
    
    strcat(folderName, "random-cubes/");
    
    pid_t pid = 1;
    auto start = chrono::high_resolution_clock::now();
    for(int i = 0; i < testNum; i ++)
    {
//        pid = fork();
//        if(pid != 0) continue;
        srand((unsigned int)time(NULL)^(i));
        
        filenameSpeed[0] = 0;
        filenameMaterial[0] = 0;
        filenameRules[0] = 0;
        filenameDiversity[0] = 0;
        strcat(filenameSpeed, folderName);
        strcat(filenameMaterial, folderName);
        strcat(filenameRules, folderName);
        strcat(filenameDiversity, folderName);

        char tmpfilenameSpeed[100];
        char tmpfilenameMaterial[100];
        char tmpfilenameRules[100];
        char tmpfilenameDiversity[100];
        snprintf(tmpfilenameSpeed, 12, "speed_%d.csv", i + 1);
        snprintf(tmpfilenameMaterial, 15, "material_%d.txt", i + 1);
        snprintf(tmpfilenameRules, 12, "rules_%d.txt", i + 1);
        snprintf(tmpfilenameDiversity, 16, "diversity_%d.txt", i + 1);

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
            
            strcat(filenameRules, "EA/");
            strcat(filenameRules, tmpfilenameRules);
            
            strcat(filenameDiversity, "EA/");
            strcat(filenameDiversity, tmpfilenameDiversity);
            evolutionAlgo();
        }
        writeSpeed();
        
//        break;
    }
    
    if(pid > 0)
    {
//        int status;
//        int tmpTestNum = testNum;
//        while (tmpTestNum --)
//        {
//            while(wait(&status) > 0);
//            printf("%d\n", status);
//        }
        auto stop = chrono::high_resolution_clock::now();
        auto duration = duration_cast<chrono::microseconds>(stop - start);
        cout << "Time taken by function: "
             << duration.count() / 1000000 << " seconds" << endl;
    }
    
    
    /* draw best robot */
    /*
    strcat(filenameRules, folderName);
    strcat(filenameRules, "EA_100_100_2/rules_4.txt");
    strcat(filenameMaterial, folderName);
    strcat(filenameMaterial, "EA_100_100_2/material_4.txt");
    bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6];
    readRules(filenameRules, rules);
    initObject(rules);
    
    struct Material materials[MAXN];
    int materialsNum = 0;
    materialsNum = readMaterial(filenameMaterial, materials);
    applyMaterialtoSprings(materials, materialsNum);
    double sp, spPath;
    speed(points, sp, spPath);
    printf("%lf %lf", sp, spPath);
    draw();
    */
    
    return 0;
}


void randomSearch()
{
    testOpenFile();
    double individualSpeed[sampleSize];
    double individualSpeedPath[sampleSize];
    int individualMaterialNum[sampleSize];
    struct Material individualMaterial[sampleSize][MAXN];
    bool individualRules[sampleSize][MAX_SIDE][MAX_SIDE][MAX_SIDE][6];

    for(int sample = 0; sample < sampleSize; sample ++)
    {
        randomRules(individualRules[sample]);
        initObject(individualRules[sample]);
        randInitMaterial(individualMaterial[sample], &individualMaterialNum[sample]);
        applyMaterialtoSprings(individualMaterial[sample], individualMaterialNum[sample]);
        speed(points, individualSpeed[sample], individualSpeedPath[sample]);
    }
    
    int bestMaterialIdx = 0;
    double maxSpeed = 0;
    double maxFitness = -1 * INFINITY;
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        for(int sample = 0; sample < sampleSize; sample ++)
        {
            randomRules(individualRules[sample]);
            initObject(individualRules[sample]);
            randInitMaterial(individualMaterial[sample], &individualMaterialNum[sample]);
            applyMaterialtoSprings(individualMaterial[sample], individualMaterialNum[sample]);
            speed(points, individualSpeed[sample], individualSpeedPath[sample]);
            
            double fitness = speedFitness(individualSpeed[sample], individualSpeedPath[sample]);
            if(fitness > maxFitness)
            {
                maxFitness = fitness;
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

static void hillClimbStep(
    struct Material *bestMaterial,
    int &bestMaterialNum,
    double &maxSpeed,
    double &maxSpeedPath,
    bool bestRules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6])
{
    bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6];
    for(int i = 0; i < MAX_SIDE; i ++)
    {
        for(int j = 0; j < MAX_SIDE; j ++)
        {
            for(int k = 0; k < MAX_SIDE; k ++)
            {
                for(int dir = 0; dir < 6; dir ++)
                {
                    rules[i][j][k][dir] = bestRules[i][j][k][dir];
                }
            }
        }
    }
    
    struct Material materials[MAXN];
    int materialsNum = bestMaterialNum;
    
    for(int i = 0; i < bestMaterialNum; i ++)
    {
        materials[i] = bestMaterial[i];
    }
    
    mutate(materials, &materialsNum, rules);
    initObject(rules);
    applyMaterialtoSprings(materials, materialsNum);
    
    double sp, spPath;
    speed(points, sp, spPath);
    
    if(speedFitness(sp, spPath) > speedFitness(maxSpeed, maxSpeedPath))
    {
        maxSpeed = sp;
        maxSpeedPath = spPath;
        
        for(int i = 0; i < MAX_SIDE; i ++)
        {
            for(int j = 0; j < MAX_SIDE; j ++)
            {
                for(int k = 0; k < MAX_SIDE; k ++)
                {
                    for(int dir = 0; dir < 6; dir ++)
                    {
                        bestRules[i][j][k][dir] = rules[i][j][k][dir];
                    }
                }
            }
        }
        
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
    testOpenFile();
    double individualSpeed[sampleSize];
    double individualSpeedPath[sampleSize];
    int individualMaterialNum[sampleSize];
    struct Material individualMaterial[sampleSize][MAXN];
    bool individualRules[sampleSize][MAX_SIDE][MAX_SIDE][MAX_SIDE][6];

    for(int sample = 0; sample < sampleSize; sample ++)
    {
        randomRules(individualRules[sample]);
        initObject(individualRules[sample]);
        randInitMaterial(individualMaterial[sample], &individualMaterialNum[sample]);
        applyMaterialtoSprings(individualMaterial[sample], individualMaterialNum[sample]);
        speed(points, individualSpeed[sample], individualSpeedPath[sample]);
    }
    
    int bestMaterialIdx = 0;
    double maxSpeed = 0;
    double maxFitness = -1 * INFINITY;
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        bestMaterialIdx = 0;
        maxSpeed = 0;
        maxFitness = -1 * INFINITY;
        for(int sample = 0; sample < sampleSize; sample ++)
        {
            hillClimbStep(individualMaterial[sample],
                          individualMaterialNum[sample],
                          individualSpeed[sample],
                          individualSpeedPath[sample],
                          individualRules[sample]);
            double fitness = speedFitness(individualSpeed[sample], individualSpeedPath[sample]);
            
            if(fitness > maxFitness)
            {
                maxFitness = fitness;
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
    bool rules[sampleSize][MAX_SIDE][MAX_SIDE][MAX_SIDE][6],
    struct Material individualMaterial[sampleSize][MAXN],
    int individualMaterialNum[sampleSize],
    double individualSpeed[sampleSize],
    double individualSpeedPath[sampleSize])
{
    for(int i = 0; i < sampleSize; i ++)
    {
        initObject(rules[i]);
        applyMaterialtoSprings(individualMaterial[i], individualMaterialNum[i]);
        speed(points, individualSpeed[i], individualSpeedPath[i]);
    }
    
    return;
}

void evolutionAlgo()
{
    testOpenFile();
    double diversity[evaluationTimes];
    double individualSpeed[sampleSize];
    double individualSpeedPath[sampleSize];
    int individualMaterialNum[sampleSize];
    struct Material individualMaterial[sampleSize][MAXN];
    bool individualRules[sampleSize][MAX_SIDE][MAX_SIDE][MAX_SIDE][6];

    for(int sample = 0; sample < sampleSize; sample ++)
    {
        randomRules(individualRules[sample]);
        initObject(individualRules[sample]);
        randInitMaterial(individualMaterial[sample], &individualMaterialNum[sample]);
        applyMaterialtoSprings(individualMaterial[sample], individualMaterialNum[sample]);
        speed(points, individualSpeed[sample], individualSpeedPath[sample]);
        printf("%lf, ", individualSpeed[sample]);
    }
    printf("\n");
    
    int bestRobotIdx = 0;
    double maxSpeed = 0;
    double maxFitness = -1 * INFINITY;
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        bestRobotIdx = 0;
        maxSpeed = 0;
        maxFitness = -1 * INFINITY;
        for(int sample = 0; sample < sampleSize; sample ++)
        {
            hillClimbStep(individualMaterial[sample],
                          individualMaterialNum[sample],
                          individualSpeed[sample],
                          individualSpeedPath[sample],
                          individualRules[sample]);
            double fitness = speedFitness(individualSpeed[sample], individualSpeedPath[sample]);
            
            if(fitness > maxFitness)
            {
                maxFitness = fitness;
                maxSpeed = individualSpeed[sample];
                bestRobotIdx = sample;
            }
            
            printf("%lf, ", individualSpeed[sample]);
        }
        printf("\n");
        bestSpeed[i] = maxSpeed;
        
        if((i + 1) % selectInterval == 0 && (i + 1) != evaluationTimes)
        {
            basicSelect(individualMaterial, individualMaterialNum, individualRules, individualSpeed, individualSpeedPath);
        }
        
        if(toWriteDiversity)
        {
            diversity[i] = getDiversity(individualMaterial, individualMaterialNum, individualRules);
        }
    }
    
    printf("speed: %lf\n", maxSpeed);
    initObject(individualRules[bestRobotIdx]);
    applyMaterialtoSprings(individualMaterial[bestRobotIdx], individualMaterialNum[bestRobotIdx]);
    draw();
//    double tmpSp, tmpSpPath;
//    speed(points, tmpSp, tmpSpPath);
//    printf("%lf %lf\n", tmpSp, tmpSpPath);
    
    FILE* fileMaterial = fopen(filenameMaterial, "w+");
    if(fileMaterial == NULL)
    {
        perror(filenameMaterial);
        exit(1);
    }
    writeMaterial(fileMaterial, individualMaterial[bestRobotIdx], individualMaterialNum[bestRobotIdx]);
    fclose(fileMaterial);
    writeRules(individualRules[bestRobotIdx]);
    writeDiversity(diversity);
    
    return;
}

/* helper methods */
void initObject(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6])
{
    generateObject(rules);
    
    return;
}

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
    else if(selectObject == 3)
    {
        initializeInsect();
    }
    else
    {
        initializeCube();
    }
    
    return;
}

void testOpenFile()
{
    FILE* fileSpeed = fopen(filenameSpeed, "w+");
    FILE* fileMaterial = fopen(filenameMaterial, "w+");
    FILE* fileRules = fopen(filenameRules, "w+");
    if(fileSpeed == NULL)
    {
        perror(filenameSpeed);
        exit(1);
    }
    if(fileMaterial == NULL)
    {
        perror(filenameMaterial);
        exit(1);
    }
    if(fileRules == NULL)
    {
        perror(filenameRules);
        exit(1);
    }
    fclose(fileSpeed);
    fclose(fileMaterial);
    fclose(fileRules);
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
        fprintf(file, "%lf %lf %lf\n", materials[i].pos[0], materials[i].pos[1], materials[i].pos[2]);
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

void writeRules(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6])
{
    FILE* fileRules = fopen(filenameRules, "w+");
    if(fileRules == NULL)
    {
        perror(filenameRules);
        exit(1);
    }
    
    fprintf(fileRules, "%d\n", MAX_SIDE);
    
    for(int i = 0; i < MAX_SIDE; i ++)
    {
        for(int j = 0; j < MAX_SIDE; j ++)
        {
            for(int k = 0; k < MAX_SIDE; k ++)
            {
                for(int dir = 0; dir < 6; dir ++)
                {
                    fprintf(fileRules, "%d ", rules[i][j][k][dir]);
                }
                fprintf(fileRules, "\n");
            }
        }
    }
    
    fclose(fileRules);
    return;
}

void readRules(char filenameRules[100], bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6])
{
    FILE* fileRules = fopen(filenameRules, "r");
    if(fileRules == NULL)
    {
        perror(filenameRules);
        exit(1);
    }
    
    int maxside;
    fscanf(fileRules, "%d\n", &maxside);
    
    for(int i = 0; i < maxside; i ++)
    {
        for(int j = 0; j < maxside; j ++)
        {
            for(int k = 0; k < maxside; k ++)
            {
                for(int dir = 0; dir < 6; dir ++)
                {
                    int rule;
                    fscanf(fileRules, "%d ", &rule);
                    rules[i][j][k][dir] = rule;
                }
                fscanf(fileRules, "\n");
            }
        }
    }
    
    fclose(fileRules);
    return;
}

void writeDiversity(double diversity[evaluationTimes])
{
    if(!toWriteDiversity) return;
    
    FILE* fileDiverse = fopen(filenameDiversity, "w+");
    if(fileDiverse == NULL)
    {
        perror(filenameDiversity);
        exit(1);
    }
    
    for(int i = 0; i < evaluationTimes; i ++)
    {
        fprintf(fileDiverse, "%lf\n", diversity[i]);
    }
    
    fclose(fileDiverse);
    return;
}

/* int[3] pos;
 * double len;
 * double k;
 * bool muscle = false;
 * double omega;
 * double b;
 * double c;
 */
int readMaterial(char filename[100], struct Material materials[MAXN])
{
    FILE* file = fopen(filenameMaterial, "r");
    if(file == NULL)
    {
        perror(filenameMaterial);
        exit(1);
    }
    
    int materialsNum = 0;
    char materialStart[15];
    while(fscanf(file, "%9c\n", materialStart) != EOF)
    {
        fscanf(file, "%lf %lf %lf\n", &materials[materialsNum].pos[0], &materials[materialsNum].pos[1], &materials[materialsNum].pos[2]);
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
