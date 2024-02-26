
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    //
    // The input screen_coord is a normalized screen coordinate [0,1]^2
    //
    // You need to transform this 2D point into a 3D position on the sensor plane, which is
    // located one unit away from the pinhole in camera space (aka view space).
    //
    // You'll need to compute this position based on the vertial field of view
    // (vert_fov) of the camera, and the aspect ratio of the output image (aspect_ratio).
    //
    // Tip: compute the ray direction in view space and use
    // the camera space to world space transform (iview) to transform the ray back into world space.

    // compute the width and height of the sensor plane
    Vec2 up_right = Vec2 {
        aspect_ratio * static_cast<float>(tan(vert_fov * PI_F / 360)),
        static_cast<float>(tan(vert_fov * PI_F / 360))
    };
    Vec2 point_on_sensor = -1.f * up_right +  2 * up_right * screen_coord;
    Vec3 o = (iview * Vec4(0.f, 0.f, 0.f, 1.f)).project();
    Vec3 d = (iview * Vec4(point_on_sensor.x, point_on_sensor.y, -1.f, 1.f)).project() - o;
    return Ray(o, d);
}
