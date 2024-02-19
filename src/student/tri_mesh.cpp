
#include "../rays/tri_mesh.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect
    Vec3 v_0 = vertex_list[v0].position;
    Vec3 v_1 = vertex_list[v1].position;
    Vec3 v_2 = vertex_list[v2].position;

    Vec3 box_min = Vec3 {std::min(std::min(v_0.x , v_1.x), v_2.x),
                        std::min(std::min(v_0.y , v_1.y), v_2.y),
                        std::min(std::min(v_0.z , v_1.z), v_2.z)};
    Vec3 box_max = Vec3 {std::max(std::max(v_0.x , v_1.x), v_2.x),
                        std::max(std::max(v_0.y , v_1.y), v_2.y),
                        std::max(std::max(v_0.z , v_1.z), v_2.z)};
         
    BBox box(box_min, box_max);
    return box;
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    // See rays/tri_mesh.h for a description of this struct
    
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    // here just to avoid unused variable warnings, students should remove the following three lines.
    
    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the above three points.
    // Intersection should yield a ray t-value, and a hit point (u,v) on the surface of the triangle

    // You'll need to fill in a "Trace" struct describing information about the hit (or lack of hit)

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
                           // (this should be interpolated between the three vertex normals)

    Vec3 e1 = v_1.position - v_0.position;
    Vec3 e2 = v_2.position - v_0.position;

    // TODO: can a trianlge with this area be hit by a ray? What does this mean?
    if (cross(e1, e2).norm() / 2 == 0) {
        // area of the trianlge is 0
        return ret;
    }
    
    Vec3 s = ray.point - v_0.position;
    Vec3 e1d = cross(e1, ray.dir);
    Vec3 nse2 = -cross(s, e2);

    if (dot(e1d, e2) == 0) {
        // meaning the e1 x d is orthogonal to e1, e2, and d, furhter meaning e1 x d is a 
        // normal vector to the triangle, and d is parallel to the triangle's plane
        // TODO: ignore?
        return ret;
    }

    Vec3 uvt = 1.f / dot(e1d, e2) * Vec3{dot(nse2, ray.dir), dot(e1d, s), dot(nse2, e1)};
    if (uvt.x < 0 || uvt.y < 0 || uvt.x + uvt.y > 1) {
        // outside the triangle
        return ret;
    }

    // check if in valid hit range 
    if (uvt.z < ray.dist_bounds.x || uvt.z > ray.dist_bounds.y) {
        return ret;
    }

    ray.dist_bounds.y = uvt.z;
    ret.hit = true;
    ret.distance = uvt.z;
    ret.position = ray.point + uvt.z * ray.dir;
    ret.normal = (1 - uvt.x - uvt.y) * v_0.normal + uvt.x * v_1.normal + uvt.y * v_2.normal;

    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
