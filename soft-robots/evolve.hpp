//
//  evolve.hpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/7.
//

#ifndef evolve_hpp
#define evolve_hpp

void applyMaterialtoSprings(struct Material* materials, int materialsNum);
void randInitMaterial(struct Material* materials, int* materialsNum);
void mutateMaterial(struct Material* materials, int* materialsNum);
void crossOver(struct Material* materials1, int* materialsNum1, struct Material* materials2, int* materialsNum2, struct Material* offspring, int* offspringNum);
double speed(struct Point*);

#endif /* evolve_hpp */
