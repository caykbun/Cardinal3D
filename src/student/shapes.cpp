
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    Trace ret;
    ret.hit = false;
    ret.origin = ray.point;

    if (radius <= 0) {
        return ret;
    }

    // Note: No center information of the sphere, seems like ray will be transformed 
    // so that the sphere is centered at the origin

    // b^2 - 4ac = 4(o \dot d)^2 - 4|d|^2(|o|^2 - r)
    float a = pow(ray.dir.norm(), 2);
    float b = 2 * dot(ray.point, ray.dir);
    float c = pow(ray.point.norm(), 2) - radius * radius;
    float test = b * b - 4 * a * c;

    if (test < 0) {
        return ret;
    }

    // solve t by the quadratic formula
    float t1 = (-b + sqrt(test)) / (2 * a);
    float t2 = (-b - sqrt(test)) / (2 * a);
    if (t1 > t2) std::swap(t1, t2);

    float intersection_t = -1;
    if (t1 >= ray.dist_bounds.x && t1 <= ray.dist_bounds.y) {
        intersection_t = t1;
    } else if (t2 != t1 && t2 >= ray.dist_bounds.x && t2 <= ray.dist_bounds.y) {
        intersection_t = t2;
    }

    if (intersection_t >= 0) {
        ret.hit = true;
        ret.distance = intersection_t;
        ret.position = ray.point + ret.distance * ray.dir;
        ret.normal = ret.position;  // normal at the intersection point is the vector from the origin to the point
        ray.dist_bounds.y = ret.distance;
    }

    return ret;
}

} // namespace PT
