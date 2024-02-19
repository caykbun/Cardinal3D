
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    if (max == min) {
        // zero-volume bounding box, treat as not hit
        return false;
    }
    
    // intersection time intervals with x, y, z planes
    float x_mint = 0;
    float x_maxt = std::numeric_limits<float>::max();
    if (ray.dir.x != 0) {
        x_mint = (min.x - ray.point.x) / ray.dir.x;
        x_maxt = (max.x - ray.point.x) / ray.dir.x;
        if (x_mint > x_maxt) std::swap(x_mint, x_maxt);
    }
    float y_mint = 0;
    float y_maxt = std::numeric_limits<float>::max();
    if (ray.dir.y != 0) {
        y_mint = (min.y - ray.point.y) / ray.dir.y;
        y_maxt = (max.y - ray.point.y) / ray.dir.y;
        if (y_mint > y_maxt) std::swap(y_mint, y_maxt);
    }
    float z_mint = 0;
    float z_maxt = std::numeric_limits<float>::max();
    if (ray.dir.z != 0) {
        z_mint = (min.z - ray.point.z) / ray.dir.z;
        z_maxt = (max.z - ray.point.z) / ray.dir.z;
        if (z_mint > z_maxt) std::swap(z_mint, z_maxt);
    }

    // overlap of the intervals
    float inter_mint = std::max(x_mint, std::max(y_mint, z_mint));
    float inter_maxt = std::min(x_maxt, std::min(y_maxt, z_maxt));
    
    // no overlap of opposite direction of the ray
    // Note: intersecting with the edges, the corners, or with a flat bbox is treated as a valid hit
    if (inter_mint > inter_maxt || (inter_mint < 0 && inter_maxt < 0)) {
        return false;
    }

    inter_mint = std::max(inter_mint, 0.f);

    if (inter_mint < times.x || inter_maxt > times.y) {
        return false;
    }

    times.x = inter_mint;
    times.y = inter_maxt;
 
    return true;
}
