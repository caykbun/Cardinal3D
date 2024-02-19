
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
// #include "bvh.h"

namespace PT {

#define NUM_BUCKETS 32

// construct BVH hierarchy given a vector of prims
template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.

    // Keep these two lines of code in your solution. They clear the list of nodes and
    // initialize member variable 'primitives' as a vector of the scene prims
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Modify the code ahead to construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.
    //
    // Please use the SAH as described in class.  We recomment the binned build from lecture.
    // In general, here is a rough sketch:
    //
    //  For each axis X,Y,Z:
    //     Try possible splits along axis, evaluate SAH for each
    //  Take minimum cost across all axes.
    //  Partition primitives into a left and right child group
    //  Compute left and right child bboxes
    //  Make the left and right child nodes.
    //
    //
    // While a BVH is conceptually a tree structure, the BVH class uses a single vector (nodes)
    // to store all the nodes. Therefore, BVH nodes don't contain pointers to child nodes,
    // but rather the indices of the
    // child nodes in this array. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.
    //
    // As an example of how to make nodes, the starter code below builds a BVH with a
    // root node that encloses all the primitives and its two descendants at Level 2.
    // For now, the split is hardcoded such that the first primitive is put in the left
    // child of the root, and all the other primitives are in the right child.
    // There are no further descendants.

    // edge case
    if(primitives.empty()) {
        return;
    }

    // compute bounding box for all primitives
    BBox bb;
    for(size_t i = 0; i < primitives.size(); ++i) {
        bb.enclose(primitives[i].bbox());
    }

    // set up root node (root BVH). Notice that it contains all primitives.
    size_t root_node_addr = new_node();
    root_idx = root_node_addr;
    Node& node = nodes[root_node_addr];
    node.bbox = bb;
    node.start = 0;
    node.size = primitives.size();

    build_helper(root_node_addr, max_leaf_size);
}

template<typename Primitive>
void BVH<Primitive>::build_helper(size_t node_addr, size_t max_leaf_size) {
    // Decide whether to conclude with a leaf node or keep splitting
    if (nodes[node_addr].size <= std::max(max_leaf_size, static_cast<size_t>(NUM_BUCKETS))) {
        return;
    };

    // Choose the best SAH split
    // compute bucket size
    Vec3 step = (nodes[node_addr].bbox.max -  nodes[node_addr].bbox.min) / NUM_BUCKETS;
    if (step == Vec3(0.f, 0.f, 0.f)) {
        // zero-volume bounding box
        return;
    }
   
    float dim_min_cost = std::numeric_limits<float>::max();
    int best_dim = 0;
    int best_split_idx = 0;
    BBox left;
    BBox right;
    size_t left_count = 0;

    // for each of x, y, z axis
    for (int dim = 0; dim < 3; dim++) {
        if (step[dim] == 0) {
            // flat box
            continue;
        }

        BBox p_bbox[NUM_BUCKETS];
        size_t p_count[NUM_BUCKETS];
        memset(p_count, 0, sizeof(size_t) * NUM_BUCKETS);

        // assign each primitive to bucket, compute bucket bounding boxes and prims count
        // TODO: can we get the actual centroid of the prim? Using center of the bbox now
        for (size_t i = nodes[node_addr].start; i < nodes[node_addr].start + nodes[node_addr].size; i++) {
            int bucket = floor((primitives[i].bbox().center()[dim] - nodes[node_addr].bbox.min[dim]) / step[dim]);
            if (bucket == NUM_BUCKETS) bucket = NUM_BUCKETS - 1;
            p_bbox[bucket].enclose(primitives[i].bbox());
            p_count[bucket]++;
        }

        // compute left and right bounding boxes under each split
        BBox p_bbox_right[NUM_BUCKETS];
        memcpy(p_bbox_right, p_bbox, sizeof(BBox) * NUM_BUCKETS);
        for (int i = 1; i < NUM_BUCKETS; i++) {
            p_bbox[i].enclose(p_bbox[i - 1]);
            p_count[i] += p_count[i - 1];
            p_bbox_right[NUM_BUCKETS - 1 - i].enclose(p_bbox_right[NUM_BUCKETS - i]);
        }
        
        // compute the SAH cost for each split and record the best split within this dimension
        float Sn = nodes[node_addr].bbox.surface_area();
        float min_cost = std::numeric_limits<float>::max();
        int best_idx = 0;
        for (int i = 0; i < NUM_BUCKETS - 1; i++) {
            size_t Na = p_count[i];
            size_t Nb = nodes[node_addr].size - Na;
            float cost = p_bbox[i].surface_area() / Sn * Na + p_bbox_right[i + 1].surface_area() / Sn * Nb;
            if (cost < min_cost) {
                min_cost = cost;
                best_idx = i;
            }
        }

        // record the best split across dimensions
        if (min_cost < dim_min_cost) {
            dim_min_cost = min_cost;
            best_dim = dim;
            best_split_idx = best_idx;
            left = p_bbox[best_idx];
            right = p_bbox_right[best_idx + 1];
            left_count = p_count[best_idx];
        }
    }

    // partition the primitives based on the best split
    float split = nodes[node_addr].bbox.min[best_dim] + (best_split_idx + 1) * step[best_dim];
    size_t first_larger = nodes[node_addr].start;
    for (size_t i = nodes[node_addr].start; i < nodes[node_addr].start + nodes[node_addr].size; i++) {
        if (primitives[i].bbox().center()[best_dim] < split) {
            std::swap(primitives[first_larger], primitives[i]);
            first_larger++;
        }
    }

    // create child nodes
    size_t node_addr_l = new_node();
    size_t node_addr_r = new_node();
    nodes[node_addr].l = node_addr_l;
    nodes[node_addr].r = node_addr_r;

    nodes[node_addr_l].bbox = left;
    nodes[node_addr_l].start = nodes[node_addr].start;
    nodes[node_addr_l].size = first_larger - nodes[node_addr].start;

    nodes[node_addr_r].bbox = right;
    nodes[node_addr_r].start = first_larger;
    nodes[node_addr_r].size = nodes[node_addr].start + nodes[node_addr].size - first_larger;

    assert(left_count == nodes[node_addr_l].size);

    // recursively build BVH
    build_helper(node_addr_l, max_leaf_size);
    build_helper(node_addr_r, max_leaf_size);
}

template<typename Primitive>
Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    ret.distance = std::numeric_limits<float>::max();
    hit_helper(ray, root_idx, ret);
    return ret;
}

template<typename Primitive>
void BVH<Primitive>::hit_helper(const Ray& ray, size_t node_addr, Trace& closest) const {
    const Node& node = nodes[node_addr];
    if (node.is_leaf()) {
        for (size_t i = node.start; i < node.start + node.size; i++){
            Trace trace = primitives[i].hit(ray);
            if (trace.hit) {
                closest = trace;
            }
        }
        return;
    }

    // compute ray intersection with the two child's bounding boxes and proceed with the closer one
    // TODO: is this how you use the time parameter?
    Vec2 hit_range_l = Vec2(0.f, std::numeric_limits<float>::max());
    bool hit_l = nodes[node.l].bbox.hit(ray, hit_range_l);
    Vec2 hit_range_r = Vec2(0.f, std::numeric_limits<float>::max());
    bool hit_r = nodes[node.r].bbox.hit(ray, hit_range_r);
    if (hit_l && hit_r) {
        size_t first = hit_range_l.x < hit_range_r.x ? node.l : node.r;
        size_t second = hit_range_l.x >= hit_range_r.x ? node.l : node.r;
        Vec2* hit_range_second = hit_range_l.x >= hit_range_r.x ? &hit_range_l : &hit_range_r;
        hit_helper(ray, first, closest);
        if (hit_range_second->x < closest.distance) {
            hit_helper(ray, second, closest);
        }
    } else if (hit_l) {
        hit_helper(ray, node.l, closest);
    } else if (hit_r) {
        hit_helper(ray, node.r, closest);
    }
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive>
BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive>
bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive>
BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive>
std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive>
void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
