#ifndef __LIGHT_H__
#define __LIGHT_H__
#include <algorithm>
#include <vector>
#include "vector.h"
#include "camera.h"
#include "Textures/texture.h"
#include "Textures/colortexture.h"


class Light{
  public:
   unsigned char* color;
   unsigned char* getColor(unsigned char a, unsigned char b, unsigned char c);
   Vector center;
   Light(const Vector & cente, unsigned char* colo);
};

struct LightNode{
   Light* data;
   LightNode* prev, *next;
};

class Shape;
struct ShapeNode{
   Shape* data;
   ShapeNode* prev, *next;
};

struct AABB {
    Vector minPt;
    Vector maxPt;
    double surfaceArea() const {
        Vector d = maxPt;
        d -= minPt;
        return 2.0 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }
    AABB() : minPt(0,0,0), maxPt(0,0,0) {}
};

inline AABB combine(const AABB& a, const AABB& b) {
    AABB r;
    r.minPt.x = std::min(a.minPt.x, b.minPt.x);
    r.minPt.y = std::min(a.minPt.y, b.minPt.y);
    r.minPt.z = std::min(a.minPt.z, b.minPt.z);
    r.maxPt.x = std::max(a.maxPt.x, b.maxPt.x);
    r.maxPt.y = std::max(a.maxPt.y, b.maxPt.y);
    r.maxPt.z = std::max(a.maxPt.z, b.maxPt.z);
    return r;
}

struct BVHNode {
    AABB bound;
    int start; 
    int end;
    BVHNode* left;
    BVHNode* right;

    BVHNode() : start(-1), end(-1), left(nullptr), right(nullptr) {}
};


class Autonoma{
public:
   Camera camera;
   Texture* skybox;
   unsigned int depth;
   ShapeNode *listStart, *listEnd;
   LightNode *lightStart, *lightEnd;
   BVHNode* bvhRoot;
   std::vector<Shape*> shapes;
   Autonoma(const Camera &c);
   Autonoma(const Camera &c, Texture* tex);
   void addShape(Shape* s);
   void removeShape(ShapeNode* s);
   void addLight(Light* s);
   void removeLight(LightNode* s);
};

void getLight(double* toFill, Autonoma* aut, Vector point, Vector norm, unsigned char r);

#endif
