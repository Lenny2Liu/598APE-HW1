#ifndef __SHAPE_H__
#define __SHAPE_H__
#include "light.h"
#include <cmath>

class Shape{
  public:
    Shape(const Vector &c, Texture* t, double ya, double pi, double ro);
    double yaw, pitch, roll, xsin, xcos, ysin, ycos, zsin, zcos;
    Vector center;
    Texture* texture;
    double textureX, textureY, mapX, mapY;
    double mapOffX = 0.0, mapOffY = 0.0;
    Texture* normalMap;
    virtual double getIntersection(Ray ray) = 0;
    virtual bool getLightIntersection(Ray ray, double* fill) = 0;
    virtual void move() = 0;
    virtual unsigned char reversible() = 0;
    virtual void getColor(unsigned char* toFill, double* am, double* op, double* ref, Autonoma* r, Ray ray, unsigned int depth) = 0;
    virtual Vector getNormal(Vector point) = 0;
    virtual void setAngles(double yaw, double pitch, double roll) = 0;
    virtual void setYaw(double d) = 0;
    virtual void setPitch(double d) = 0;
    virtual void setRoll(double d) = 0;
    virtual void getAABB(AABB& outBox) const = 0;
    virtual ~Shape();
};

struct ShapeInfo {
    Shape* shape; 
    AABB   bound; 
};

BVHNode* buildBVH(std::vector<ShapeInfo>& shapeInfos, int start, int end, int depth, int maxDepth, int minLeaf);
void calcColor(unsigned char* toFill, Autonoma*, Ray ray, unsigned int depth);

#endif
