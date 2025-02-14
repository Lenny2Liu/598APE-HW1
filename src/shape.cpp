#include "shape.h"
#include <algorithm>
#include <vector>
#include <limits>
#include <iostream>

Shape::Shape(const Vector &c, Texture* t, double ya, double pi, double ro): center(c), texture(t), yaw(ya), pitch(pi), roll(ro){
};

void Shape::setAngles(double a, double b, double c){
   yaw =a; pitch = b; roll = c;
   xcos = cos(yaw);
   xsin = sin(yaw);
   ycos = cos(pitch);
   ysin = sin(pitch);
   zcos = cos(roll);
   zsin = sin(roll);
}

void Shape::setYaw(double a){
   yaw =a;
   xcos = cos(yaw);
   xsin = sin(yaw);
}

void Shape::setPitch(double b){
   pitch = b;
   ycos = cos(pitch);
   ysin = sin(pitch);
}

void Shape::setRoll(double c){
   roll = c;
   zcos = cos(roll);
   zsin = sin(roll);
}

typedef struct {
   double time;
   Shape* shape;
} TimeAndShape;

void insertionSort(TimeAndShape *arr, int n) {
    for (int i = 1; i < n; ++i) {
        TimeAndShape key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j].time > key.time) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }
}

// void calcColor(unsigned char* toFill,Autonoma* c, Ray ray, unsigned int depth){
//    ShapeNode* t = c->listStart;
//    TimeAndShape *times = (TimeAndShape*)malloc(0);
//    size_t seen = 0;
//    while(t!=NULL){
//       double time = t->data->getIntersection(ray);

//       TimeAndShape *times2 = (TimeAndShape*)malloc(sizeof(TimeAndShape)*(seen + 1));
//       for (int i=0; i<seen; i++)
//          times2[i] = times[i];
//       times2[seen] = (TimeAndShape){ time, t->data };
//       free(times);
//       times = times2;
//       seen ++;
//       t = t->next;
//    }
//    insertionSort(times, seen);
//    if (seen == 0 || times[0].time == inf) {
//       double opacity, reflection, ambient;
//       Vector temp = ray.vector.normalize();
//       const double x = temp.x;
//       const double z = temp.z;
//       const double me = (temp.y<0)?-temp.y:temp.y;
//       const double angle = atan2(z, x);
//       c->skybox->getColor(toFill, &ambient, &opacity, &reflection, fix(angle/M_TWO_PI),fix(me));
//       return;
//    }

//    double curTime = times[0].time;
//    Shape* curShape = times[0].shape;
//    free(times);

//    Vector intersect = curTime*ray.vector+ray.point;
//    double opacity, reflection, ambient;
//    curShape->getColor(toFill, &ambient, &opacity, &reflection, c, Ray(intersect, ray.vector), depth);
   
//    double lightData[3];
//    getLight(lightData, c, intersect, curShape->getNormal(intersect), curShape->reversible());
//    toFill[0] = (unsigned char)(toFill[0]*(ambient+lightData[0]*(1-ambient)));
//    toFill[1] = (unsigned char)(toFill[1]*(ambient+lightData[1]*(1-ambient)));
//    toFill[2] = (unsigned char)(toFill[2]*(ambient+lightData[2]*(1-ambient)));
//    if(depth<c->depth && (opacity<1-1e-6 || reflection>1e-6)){
//       unsigned char col[4];
//       if(opacity<1-1e-6){
//          Ray nextRay = Ray(intersect+ray.vector*1E-4, ray.vector);
//          calcColor(col, c, nextRay, depth+1);
//          toFill[0]= (unsigned char)(toFill[0]*opacity+col[0]*(1-opacity));
//          toFill[1]= (unsigned char)(toFill[1]*opacity+col[1]*(1-opacity));
//          toFill[2]= (unsigned char)(toFill[2]*opacity+col[2]*(1-opacity));        
//       }
//       if(reflection>1e-6){
//          Vector norm = curShape->getNormal(intersect).normalize();
//          Vector vec = ray.vector-2*norm*(norm.dot(ray.vector));
//          Ray nextRay = Ray(intersect+vec*1E-4, vec);
//          calcColor(col, c, nextRay, depth+1);
      
//          toFill[0]= (unsigned char)(toFill[0]*(1-reflection)+col[0]*(reflection));
//          toFill[1]= (unsigned char)(toFill[1]*(1-reflection)+col[1]*(reflection));
//          toFill[2]= (unsigned char)(toFill[2]*(1-reflection)+col[2]*(reflection));
//       }
//    }
// }


inline bool intersectAABB(const AABB& box, const Ray& ray) {
    double tMin = -inf;
    double tMax = inf;

    for(int i=0; i<3; i++){
        double invD = 1.0 / ( (i==0) ? ray.vector.x : (i==1 ? ray.vector.y : ray.vector.z) );
        double t0   = ( ((i==0)? box.minPt.x : (i==1? box.minPt.y : box.minPt.z)) - ((i==0)? ray.point.x : (i==1? ray.point.y : ray.point.z)) ) * invD;
        double t1   = ( ((i==0)? box.maxPt.x : (i==1? box.maxPt.y : box.maxPt.z)) - ((i==0)? ray.point.x : (i==1? ray.point.y : ray.point.z)) ) * invD;

        if(t0>t1) std::swap(t0, t1);
        if(t0>tMin) tMin = t0;
        if(t1<tMax) tMax = t1;
        if(tMax < tMin) return false;
    }
    return true;
}


BVHNode* buildBVH(std::vector<ShapeInfo>& shapeInfos, int start, int end)
{
    BVHNode* node = new BVHNode();
    AABB box;

    box.minPt = Vector( inf,  inf,  inf);
    box.maxPt = Vector(-inf, -inf, -inf);

    for(int i = start; i < end; i++) {
        box = combine(box, shapeInfos[i].bound);
    }
    node->bound = box;

    int count = end - start;

    if(count <= 2) {
        node->start = start;
        node->end   = end;
        node->left  = nullptr;
        node->right = nullptr;
        return node;
    }

    Vector size = box.maxPt - box.minPt;
    int axis = 0;
    if(size.y > size.x && size.y > size.z) axis = 1;
    else if(size.z > size.x && size.z > size.y) axis = 2;

    double mid = 0.0;
    for(int i = start; i<end; i++){
        AABB &b = shapeInfos[i].bound;
        double c = 0.5 * ((axis == 0 ? b.minPt.x : (axis == 1 ? b.minPt.y : b.minPt.z)) + 
                        (axis == 0 ? b.maxPt.x : (axis == 1 ? b.maxPt.y : b.maxPt.z)));
        mid += c;
    }
    mid /= count;

    auto pivot = std::partition(shapeInfos.begin() + start, shapeInfos.begin() + end,
        [&](const ShapeInfo &si){
            double c = 0.5 * ((axis == 0 ? si.bound.minPt.x : (axis == 1 ? si.bound.minPt.y : si.bound.minPt.z)) + 
                            (axis == 0 ? si.bound.maxPt.x : (axis == 1 ? si.bound.maxPt.y : si.bound.maxPt.z)));
            return c < mid;
        }
    );
    int m = (int)(pivot - shapeInfos.begin());

    if(m == start || m == end) {
        m = start + (count / 2);
    }

    node->left  = buildBVH(shapeInfos, start, m);
    node->right = buildBVH(shapeInfos, m, end);
    node->start = -1;
    node->end   = -1;
    return node;
}



bool intersectBVH(const BVHNode* node, const Ray& ray, double& outT, Shape*& outShape,
                  const std::vector<Shape*>& shapes)
{
    
    if(!node) return false;
    if(!intersectAABB(node->bound, ray)) {
        // std::cout << node << std::endl;
        return false;
    }
    // std::cout << node << std::endl;
    
    if(node->left == nullptr && node->right == nullptr) {
        bool hit = false;
        for(int i = node->start; i<node->end; i++){
            Shape* s = shapes[i];
            double t = s->getIntersection(ray);
            // std::cout << t << std::endl;
            if(t > 1e-6 && t < outT){
                // std::cout << t << std::endl;
                outT     = t;
                outShape = s;
                hit      = true;
            }
        }
        return hit;
    }

    bool hitLeft  = intersectBVH(node->left,  ray, outT, outShape, shapes);
    bool hitRight = intersectBVH(node->right, ray, outT, outShape, shapes);
    return hitLeft || hitRight;
}


void calcColor(unsigned char* toFill, Autonoma* c, const Ray ray, unsigned int depth)
{
    double closestT = inf;
    Shape* closestShape = nullptr;
    bool hit = intersectBVH(c->bvhRoot, ray, closestT, closestShape, c->shapes);
    if(!hit || closestT == std::numeric_limits<double>::infinity()) {

        Vector temp = ray.vector;
        temp.normalize();
        double x = temp.x, z = temp.z;
        double me = (temp.y < 0.0)? -temp.y : temp.y;
        double angle = atan2(z, x);

        double opacity, reflection, ambient;
        c->skybox->getColor(toFill, &ambient, &opacity, &reflection,
                            fix(angle/(2.0 * M_PI)),
                            fix(me));
        return;
    }

    Vector intersect =  closestT * ray.vector + ray.point;
    double opacity, reflection, ambient;
    closestShape->getColor(toFill, &ambient, &opacity, &reflection,
                           c, Ray(intersect, ray.vector),
                           depth);

    double lightData[3];
    {
        Vector normal = closestShape->getNormal(intersect);
        getLight(lightData, c, intersect, normal, closestShape->reversible());
    }

    toFill[0] = (unsigned char)( toFill[0] * (ambient + lightData[0]*(1 - ambient)) );
    toFill[1] = (unsigned char)( toFill[1] * (ambient + lightData[1]*(1 - ambient)) );
    toFill[2] = (unsigned char)( toFill[2] * (ambient + lightData[2]*(1 - ambient)) );

    if(depth < c->depth && (opacity < 1.0 - 1e-6 || reflection > 1e-6)) {
        unsigned char col[4];
        if(opacity < 1.0 - 1e-6) {
            Ray nextRay(intersect + 1e-4 * ray.vector, ray.vector);
            calcColor(col, c, nextRay, depth+1);
            toFill[0] = (unsigned char)( toFill[0]*opacity + col[0]*(1 - opacity) );
            toFill[1] = (unsigned char)( toFill[1]*opacity + col[1]*(1 - opacity) );
            toFill[2] = (unsigned char)( toFill[2]*opacity + col[2]*(1 - opacity) );
        }
        if(reflection > 1e-6) {
            Vector normal = closestShape->getNormal(intersect).normalize();
            double dotVN   = normal.dot(ray.vector);
            Vector reflVec = - normal*(2.0*dotVN) + ray.vector;

            Ray reflRay(intersect + reflVec*1e-4, reflVec);
            calcColor(col, c, reflRay, depth+1);

            toFill[0] = (unsigned char)( toFill[0]*(1 - reflection) + col[0]*reflection );
            toFill[1] = (unsigned char)( toFill[1]*(1 - reflection) + col[1]*reflection );
            toFill[2] = (unsigned char)( toFill[2]*(1 - reflection) + col[2]*reflection );
        }
    }
}