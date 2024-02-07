
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    HalfedgeRef start = e->halfedge();

    // the start and the end halfedges of the faces on the two sides of e
    HalfedgeRef side_starts[2] = {start, start->twin()};
    HalfedgeRef side_lasts[2];

    for(int i = 0; i < 2; i++) {
        HalfedgeRef start = side_starts[i];
        HalfedgeRef last = start;
        int edges = 1;
        while(last->next() != start) {
            edges++;
            last = last->next();
        }

        // Reduce to the polygon case this face is a triangle
        if(edges == 3) {
            FaceRef face_to_erase = start->face();
            start->face() = last->twin()->face();

            start->next()->next() = last->twin()->next();
            start->next()->face() = last->twin()->face();

            HalfedgeRef last_from_twin = last->twin();
            while(last_from_twin->next() != last->twin()) {
                last_from_twin = last_from_twin->next();
            }
            last_from_twin->next() = start;

            // erase face
            erase(face_to_erase);

            // erase edge
            last->vertex()->halfedge() = last->twin()->next();
            last->twin()->vertex()->halfedge() = start;
            last->twin()->face()->halfedge() = last->twin()->next();
            erase(last->edge());
            erase(last->twin());
            erase(last);

            // record the last halfedge of the resulting polygon
            side_lasts[i] = last_from_twin;
        } else {
            side_lasts[i] = last;
        }
    }

    // collapsing edge
    // attach halfedges on vertex_delete to vertex_keep
    HalfedgeRef cur = start->twin()->next();
    while(cur != start) {
        cur->vertex() = side_starts[1]->vertex();
        cur = cur->twin()->next();
    }

    // rewire halfedges for the two faces
    side_lasts[0]->next() = side_starts[0]->next();
    side_lasts[1]->next() = side_starts[1]->next();

    // update halfedge references of the elements affected by the erasion of the edge
    side_starts[0]->face()->halfedge() = side_starts[0]->next();
    side_starts[1]->face()->halfedge() = side_starts[1]->next();
    side_starts[1]->vertex()->halfedge() = side_starts[0]->next();

    // recompute vertex_keep's position
    side_starts[1]->vertex()->pos =
        (side_starts[1]->vertex()->center() + side_starts[0]->vertex()->center()) / 2.f;
    VertexRef vertex_keep = side_starts[1]->vertex();

    // erase edge and vertex_delete
    erase(side_starts[0]->edge());
    erase(side_starts[0]->vertex());
    erase(side_starts[0]);
    erase(side_starts[1]);

    return vertex_keep;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    HalfedgeRef cur_he = e->halfedge();
    HalfedgeRef twin_he = cur_he->twin();

    // change current halfedge
    HalfedgeRef old_next = cur_he->next();
    cur_he->next() = cur_he->next()->next();
    if(cur_he->vertex()->halfedge() == cur_he) {
        cur_he->vertex()->halfedge() = twin_he->next();
    }
    cur_he->vertex() = twin_he->next()->next()->vertex();

    // change twin halfedge
    HalfedgeRef old_twin_next = twin_he->next();
    twin_he->next() = twin_he->next()->next();
    if(twin_he->vertex()->halfedge() == twin_he) {
        twin_he->vertex()->halfedge() = old_next;
    }
    twin_he->vertex() = old_next->next()->vertex();

    // change original next halfedge
    if(old_next->face()->halfedge() == old_next) {
        old_next->face()->halfedge() = old_next->next();
    }
    old_next->face() = twin_he->face();
    old_next->next() = twin_he;

    // chnage original before halfedge
    HalfedgeRef before = cur_he;
    do {
        before = before->next();
    } while(before->next() != cur_he);
    before->next() = old_twin_next;

    // change original twin's next halfedge
    if(old_twin_next->face()->halfedge() == old_twin_next) {
        old_twin_next->face()->halfedge() = old_twin_next->next();
    }
    old_twin_next->face() = cur_he->face();
    old_twin_next->next() = cur_he;

    // change original twin's before halfedge
    before = twin_he;
    do {
        before = before->next();
    } while(before->next() != twin_he);
    before->next() = old_next;

    // TODO: when to return null?
    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    // Check if the edge is on the boundary
    if(e->on_boundary()) {
        /*
               c          c
             / |        / |
            a  |  -->  a--m
             \ |        \ |
               b          b
        */
        HalfedgeRef bc = e->halfedge();
        if(bc->is_boundary()) {
            bc = bc->twin(); // make sure bc is not a boundary halfedge
        }
        HalfedgeRef ca = bc->next();
        HalfedgeRef ab = ca->next();
        HalfedgeRef cb = bc->twin();

        // only work for triangle mesh
        if(ab->next() != bc) {
            return std::nullopt;
        }

        // create new halfedges
        HalfedgeRef mc = new_halfedge();
        HalfedgeRef cm = new_halfedge();
        HalfedgeRef mb = new_halfedge();
        HalfedgeRef bm = new_halfedge();
        HalfedgeRef ma = new_halfedge();
        HalfedgeRef am = new_halfedge();

        // create new vertices
        VertexRef m = new_vertex();
        m->pos = (bc->vertex()->pos + cb->vertex()->pos) / 2.f;

        // create new edges
        EdgeRef top_edge = new_edge();
        EdgeRef bottom_edge = new_edge();
        EdgeRef middle_edge = new_edge();

        // create new faces
        FaceRef top_face = new_face();
        FaceRef bottom_face = new_face();

        // rewire halfedges
        mc->vertex() = m;
        mc->edge() = top_edge;
        mc->face() = top_face;
        mc->next() = ca;
        mc->twin() = cm;

        cm->vertex() = ca->vertex();
        cm->edge() = top_edge;
        cm->face() = cb->face();
        cm->next() = mb;
        cm->twin() = mc;

        mb->vertex() = m;
        mb->edge() = bottom_edge;
        mb->face() = cb->face();
        mb->next() = cb->next();
        mb->twin() = bm;

        bm->vertex() = bc->vertex();
        bm->edge() = bottom_edge;
        bm->face() = bottom_face;
        bm->next() = ma;
        bm->twin() = mb;

        ma->vertex() = m;
        ma->edge() = middle_edge;
        ma->face() = bottom_face;
        ma->next() = ab;
        ma->twin() = am;

        am->vertex() = ab->vertex();
        am->edge() = middle_edge;
        am->face() = top_face;
        am->next() = mc;
        am->twin() = ma;

        ca->face() = top_face;
        ca->next() = am;

        ab->face() = bottom_face;
        ab->next() = bm;

        // find the previous halfedge of cb
        unsigned int boundary_face_degree = cb->face()->degree();
        HalfedgeRef prev = cb;
        for(unsigned int i = 0; i < boundary_face_degree - 1; i++) {
            prev = prev->next();
        }
        prev->next() = cm;

        // rewire vertices
        m->halfedge() = mc;
        ca->vertex()->halfedge() = ca;
        ab->twin()->vertex()->halfedge() = ab->twin();

        // rewire edges
        top_edge->halfedge() = mc;
        bottom_edge->halfedge() = bm;
        middle_edge->halfedge() = ma;

        // rewire faces
        top_face->halfedge() = mc;
        bottom_face->halfedge() = ma;
        cb->face()->halfedge() = cm;

        // remove old halfedges, edges and faces
        FaceRef old_face = bc->face();
        erase(bc);
        erase(cb);
        erase(e);
        erase(old_face);

        return m;
    } else {
        /*
               c             c
             / | \         / | \
            a  |  d  -->  a--m--d
             \ | /         \ | /
               b             b
        */
        HalfedgeRef bc = e->halfedge();
        HalfedgeRef ca = bc->next();
        HalfedgeRef ab = ca->next();
        HalfedgeRef cb = bc->twin();
        HalfedgeRef bd = cb->next();
        HalfedgeRef dc = bd->next();

        // only work for triangle mesh
        if(ab->next() != bc || dc->next() != cb) {
            return std::nullopt;
        }

        // create new halfedges
        HalfedgeRef mc = new_halfedge();
        HalfedgeRef cm = new_halfedge();
        HalfedgeRef md = new_halfedge();
        HalfedgeRef dm = new_halfedge();
        HalfedgeRef mb = new_halfedge();
        HalfedgeRef bm = new_halfedge();
        HalfedgeRef ma = new_halfedge();
        HalfedgeRef am = new_halfedge();

        // create new vertices
        VertexRef m = new_vertex();
        m->pos = (bc->vertex()->pos + cb->vertex()->pos) / 2.f;

        // create new edges
        EdgeRef top_edge = new_edge();
        EdgeRef bottom_edge = new_edge();
        EdgeRef left_edge = new_edge();
        EdgeRef right_edge = new_edge();

        // create new faces
        FaceRef top_left_face = new_face();
        FaceRef top_right_face = new_face();
        FaceRef bottom_left_face = new_face();
        FaceRef bottom_right_face = new_face();

        // rewire halfedges
        mc->vertex() = m;
        mc->edge() = top_edge;
        mc->face() = top_left_face;
        mc->next() = ca;
        mc->twin() = cm;

        cm->vertex() = ca->vertex();
        cm->edge() = top_edge;
        cm->face() = top_right_face;
        cm->next() = md;
        cm->twin() = mc;

        md->vertex() = m;
        md->edge() = right_edge;
        md->face() = top_right_face;
        md->next() = dc;
        md->twin() = dm;

        dm->vertex() = dc->vertex();
        dm->edge() = right_edge;
        dm->face() = bottom_right_face;
        dm->next() = mb;
        dm->twin() = md;

        mb->vertex() = m;
        mb->edge() = bottom_edge;
        mb->face() = bottom_right_face;
        mb->next() = bd;
        mb->twin() = bm;

        bm->vertex() = bd->vertex();
        bm->edge() = bottom_edge;
        bm->face() = bottom_left_face;
        bm->next() = ma;
        bm->twin() = mb;

        ma->vertex() = m;
        ma->edge() = left_edge;
        ma->face() = bottom_left_face;
        ma->next() = ab;
        ma->twin() = am;

        am->vertex() = ab->vertex();
        am->edge() = left_edge;
        am->face() = top_left_face;
        am->next() = mc;
        am->twin() = ma;

        ca->face() = top_left_face;
        ca->next() = am;

        ab->face() = bottom_left_face;
        ab->next() = bm;

        bd->face() = bottom_right_face;
        bd->next() = dm;

        dc->face() = top_right_face;
        dc->next() = cm;

        // rewire vertices
        m->halfedge() = mc;
        ca->vertex()->halfedge() = cm;
        bd->vertex()->halfedge() = bm;

        // rewire edges
        top_edge->halfedge() = mc;
        bottom_edge->halfedge() = mb;
        left_edge->halfedge() = ma;
        right_edge->halfedge() = md;

        // rewire faces
        top_left_face->halfedge() = mc;
        top_right_face->halfedge() = md;
        bottom_left_face->halfedge() = ma;
        bottom_right_face->halfedge() = mb;

        // remove old halfedges, edges and faces
        FaceRef old_left_face = bc->face();
        FaceRef old_right_face = cb->face();
        erase(bc);
        erase(cb);
        erase(e);
        erase(old_left_face);
        erase(old_right_face);

        return m;
    }
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)f;
    return std::nullopt;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {
    for(FaceRef face = faces_begin(); face != faces_end(); face++) {
        HalfedgeRef prev_out = face->halfedge();
        VertexRef origin = prev_out->vertex();

        while(prev_out->next()->next()->next() != face->halfedge()) {
            // Make new elements and wire up
            HalfedgeRef back = new_halfedge();
            HalfedgeRef out = new_halfedge();
            EdgeRef e = new_edge();
            FaceRef f = new_face();
            back->edge() = e;
            back->face() = f;
            back->next() = prev_out;
            back->twin() = out;
            back->vertex() = prev_out->next()->next()->vertex();
            out->edge() = e;
            out->next() = prev_out->next()->next();
            out->twin() = back;
            out->vertex() = origin;
            e->halfedge() = back;
            f->halfedge() = back;

            // Connect with existing edges
            prev_out->face() = f;
            prev_out->next()->next() = back;
            prev_out->next()->face() = f;

            prev_out = out;
        }

        // handle the last triangle
        prev_out->next()->next()->next() = prev_out;
        prev_out->face() = face;
        face->halfedge() = prev_out;
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->new_pos = (e->halfedge()->vertex()->pos + e->halfedge()->twin()->vertex()->pos) / 2.f;
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {
        HalfedgeRef cur = f->halfedge();
        int num_edges = 0;
        f->new_pos = {};
        do {
            num_edges++;
            f->new_pos += cur->vertex()->pos;
            cur = cur->next();
        } while(cur != f->halfedge());
        f->new_pos /= num_edges * 1.f;
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces

    // Edges

    // Vertices
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
