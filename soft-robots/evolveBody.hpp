//
//  evolveBody.hpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/17.
//

#ifndef evolveBody_hpp
#define evolveBody_hpp

void generateObject(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void generateOccupancy(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6], bool occupancy[MAX_SIDE][MAX_SIDE][MAX_SIDE]);
void randomRules(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void printRules(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);

#endif /* evolveBody_hpp */
