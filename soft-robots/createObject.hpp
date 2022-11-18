//
//  createObject.hpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/7.
//

#ifndef createObject_hpp
#define createObject_hpp

void initializeCube();
void initializeTetrahedral();
void initializeTwoCubes();
void initializeWalkingCubes();
void initializeInsect();
void initializeSpringsForEachCube(int pIdx, struct Point* points[], int numPoints);
double calcDist(double p1[3], double p2[3]);

#endif /* createObject_hpp */
