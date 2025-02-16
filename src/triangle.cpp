#include "triangle.h"

Triangle::Triangle(Vector c, Vector b, Vector a, Texture* t):Plane(Vector(0,0,0), t, 0., 0., 0., 0., 0.){
   ax = a.x; ay = a.y; az = a.z;
   bx = b.x; by = b.y; bz = b.z;
   cx = c.x; cy = c.y; cz = c.z;
   center = c;
   Vector righta = (b-c);
   textureX = righta.mag();
   right = righta/textureX;
   vect = right.cross(b-a).normalize();

   xsin = -right.z;
   if(xsin<-1.)xsin = -1;
   else if (xsin>1.)xsin=1.; 
   yaw = asin(xsin);
   xcos = sqrt(1.-xsin*xsin);

   zcos = right.x/xcos;
   zsin = -right.y/xcos;
   if(zsin<-1.)zsin = -1;
   else if (zsin>1.)zsin=1.;
   if(zcos<-1.)zcos = -1;
   else if (zcos>1.)zcos=1.;
   roll = asin(zsin);

   ycos = vect.z/xcos;
   if(ycos<-1.)ycos = -1;
   else if (ycos>1.)ycos=1.;
   pitch = acos(ycos);
   ysin = sqrt(1-ycos*ycos);

   up.x = -xsin*ysin*zcos+ycos*zsin;
   up.y = ycos*zcos+xsin*ysin*zsin;
   up.z = -xcos*ysin;
   Vector temp = vect.cross(right);
   Vector np = solveScalers(right, up, vect, a-c);
   textureY = np.y;
   thirdX = np.x;
   
   d = -vect.dot(center);
}



double Triangle::getIntersection(Ray ray) {
   double time = Plane::getIntersection(ray);
   if(time==inf) 
      return time;
   Vector dist = solveScalers(right, up, vect, ray.point+ray.vector*time-center); 
   unsigned char tmp = (thirdX - dist.x) * textureY + (thirdX-textureX) * (dist.y - textureY) < 0.0;
   return((tmp!=(textureX * dist.y < 0.0)) || (tmp != (dist.x * textureY - thirdX * dist.y < 0.0)))?inf:time;
}


// double Triangle::getIntersection(Ray ray) {
//    const double EPSILON = 1e-6;
//    Vector v0 = Vector(ax, ay, az);
//    Vector v1 = Vector(bx, by, bz);
//    Vector v2 = Vector(cx, cy, cz);
//    Vector edge1 = v1 - v0;
//    Vector edge2 = v2 - v0;
   
//    Vector h = ray.vector.cross(edge2);
//    double a = edge1.dot(h);
//    if (fabs(a) < EPSILON)
//       return inf; 
   
//    double f = 1.0 / a;
//    Vector s = - v0 + ray.point;
//    double u = f * s.dot(h);
//    if (u < 0.0 || u > 1.0)
//       return inf;
   
//    Vector q = s.cross(edge1);
//    double v = f * ray.vector.dot(q);
//    if (v < 0.0 || (u + v) > 1.0)
//       return inf;
   
//    double t = f * edge2.dot(q);
//    return (t > EPSILON) ? t : inf;
// }

bool Triangle::getLightIntersection(Ray ray, double* fill){
   const double t = ray.vector.dot(vect);
   const double norm = vect.dot(ray.point)+d;
   const double r = -norm/t;
   if(r<=0. || r>=1.) return false;
   Vector dist = solveScalers(right, up, vect, ray.point+ray.vector*r-center);
   
   unsigned char tmp = (thirdX - dist.x) * textureY + (thirdX-textureX) * (dist.y - textureY) < 0.0;
   if ((tmp!=(textureX * dist.y < 0.0)) || (tmp != (dist.x * textureY - thirdX * dist.y < 0.0))) return false;
   
   if(texture->opacity>1-1E-6) return true;   
   unsigned char temp[4];
   double amb, op, ref;
   texture->getColor(temp, &amb, &op, &ref,fix(dist.x/textureX-.5), fix(dist.y/textureY-.5));
   if(op>1-1E-6) return true;
   fill[0]*=temp[0]/255.;
   fill[1]*=temp[1]/255.;
   fill[2]*=temp[2]/255.;
   return false;
}
