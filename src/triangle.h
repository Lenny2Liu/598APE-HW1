#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__
#include "plane.h"
#include <iostream>

class Triangle : public Plane{
public:
   double thirdX;
   double ax, ay, az, bx, by, bz, cx, cy, cz;
   Triangle(Vector c, Vector b, Vector a, Texture* t);
   double getIntersection(Ray ray);
   bool getLightIntersection(Ray ray, double* fill) override;

};

#endif
