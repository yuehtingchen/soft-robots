//
//  draw.hpp
//  soft-robots
//
//  Created by Alice Chen on 2022/10/29.
//

#ifndef setPoints_hpp
#define setPoints_hpp

void intializeWalkingCubes();
void initializePointsCube();
void initializePointsTetrahedral();
void initializeSprings();
void updatePoints();
void writeEnergy();
void printPoints();
float calcPotentialEnergy();
float calcKineticEnergy();

#endif /* draw_hpp */
