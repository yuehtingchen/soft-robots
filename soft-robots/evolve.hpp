//
//  evolve.hpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/7.
//

#ifndef evolve_hpp
#define evolve_hpp
#include "utility.h"

void applyMaterialtoSprings(struct Material* materials, int materialsNum);
void randInitMaterial(struct Material* materials, int* materialsNum);
void mutateBody(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void mutateMaterial(struct Material* materials, int* materialsNum);
void mutate(struct Material materials[MAXN], int* materialsNum, bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void crossOver(bool rules1[MAX_SIDE][MAX_SIDE][MAX_SIDE][6], bool rules2[MAX_SIDE][MAX_SIDE][MAX_SIDE][6], bool rulesOffspring[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void crossOver(struct Material* materials1, int* materialsNum1, struct Material* materials2, int* materialsNum2, struct Material* offspring, int* offspringNum);
void basicSelect(struct Material materials[sampleSize][MAXN], int materialsNum[sampleSize], bool rules[sampleSize][MAX_SIDE][MAX_SIDE][MAX_SIDE][6], double sp[sampleSize], double speedPath[sampleSize]);
void speed(struct Point points[MAXN], double& speed, double& speedPath);
double speedFitness(double speed, double speedPath);
void getCenterOfMass(struct Point points[MAXN], double centerPos[3]);
double getDiversity(struct Material materials[][MAXN], int materialsNum[], bool rules[][MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void printMaterials(struct Material materials[MAXN], int materialsNum);
int random(int low, int high);

#endif /* evolve_hpp */
