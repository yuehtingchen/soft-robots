//
//  evolve.hpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/7.
//

#ifndef evolve_hpp
#define evolve_hpp

void initSpringsMaterial(struct Material* materials, int materialsNum);
void randInitMaterial(struct Material* materials, int* materialsNum);
void mutateMaterial(struct Material* materials, int* materialsNum);
double speed(struct Point*);

#endif /* evolve_hpp */
