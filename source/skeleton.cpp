//
// Skeleton class to wrap CGAL Straight-Skeleton library.
//
#include "lxsdk/lxresult.h"
#include "lxsdk/lxvmath.h"
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxu_matrix.hpp>

#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/extrude_skeleton.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

#include "skeleton.hpp"

//
// Straight skeleton class.
//
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
 
typedef K::FT                           FT;
typedef K::Point_2                      Point_2;
typedef CGAL::Polygon_2<K>              Polygon_2;
typedef CGAL::Straight_skeleton_2<K>    Ss;
 
typedef std::shared_ptr<Polygon_2>      PolygonPtr;
typedef std::shared_ptr<Ss>             SsPtr;
 
typedef std::vector<PolygonPtr>         PolygonPtrVector;

typedef CGAL::Polygon_with_holes_2<K>   Polygon_with_holes_2;
typedef CGAL::Surface_mesh<K::Point_3>  SurfaceMesh;

typedef Polygon_2::Vertex_iterator      VertexIterator;

typedef std::shared_ptr<Polygon_with_holes_2> PolygonWithHolesPtr ;
typedef std::vector<PolygonWithHolesPtr> PolygonWithHolesPtrVector;

// Custom Hash Function
struct VertexIteratorHash {
    std::size_t operator()(const VertexIterator& it) const {
        // ポインタのアドレスをハッシュ化
        return std::hash<const void*>()(&(*it));
    }
};

// Custom Compare Function
struct VertexIteratorEqual {
    bool operator()(const VertexIterator& it1, const VertexIterator& it2) const {
        return &(*it1) == &(*it2);
    }
};

//
// Construct a straight skeleton and subdivide the polygon.
//
LxResult CSkeleton::Skeleton(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols)
{
    CLxUser_Point point, point1;
    point.fromMesh(m_mesh);
    point1.fromMesh(m_mesh);

    LXtVector norm;
    polygon.Normal(norm);

    // Set axis plane to compute the triangulation on 2D space.
    AxisPlane axisPlane(norm);

    std::vector<LXtPointID> source;

    std::vector<std::vector<LXtPointID>> loops;
    MeshUtil::MakeBoundaryVertexList(m_mesh, polygon, source, loops);

    Polygon_with_holes_2 poly;

    double   z_ave = 0.0;
    for (auto i = 0u; i < source.size(); i++)
    {
        point.Select(source[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        poly.outer_boundary().push_back(Point_2(x, y));
        z_ave += z;
    }

    // averaged z value on axis plane
    z_ave /= static_cast<double>(source.size());

    for (auto& loop : loops)
    {
        Polygon_2 hole;
        for (auto i = 0u; i < loop.size(); i++)
        {
            point.Select(loop[i]);
            LXtFVector pos;
            point.Pos(pos);
            double x, y, z;
            axisPlane.ToPlane(pos, x, y, z);
            hole.push_back(Point_2(x, y));
        }
        poly.add_hole(hole);
    }

    SsPtr skeleton = CGAL::create_interior_straight_skeleton_2(poly);
    if (!skeleton)
        return LXe_FAILED;

    std::unordered_map<Ss::Vertex_const_handle, LXtPointID> vertex_to_pntID;

    AxisTriangles axisTriangles(m_mesh, polygon);
    std::vector<double> weights;

    int i = 0;
    for (auto v = skeleton->vertices_begin(); v != skeleton->vertices_end(); v++)
    {
        LXtPointID pntID;
        LXtVector  pos;
 //       printf("[%d] x = %f, y = %f contour (%d) skeleton (%d) split (%d)\n", 
 //           i++, v->point().x(), v->point().y(), v->is_contour(), v->is_skeleton(), v->is_split());
        axisPlane.FromPlane(pos, v->point().x(), v->point().y(), z_ave);
        axisTriangles.MakePositionWeights(pos, weights);
        if (v->id() < source.size())
            pntID = source[v->id()];
        else
            m_poledit.AddFaceVertex(pos, polygon.ID(), weights.data(), &pntID);
        vertex_to_pntID.insert(std::make_pair(v, pntID));
    }

    for (auto f = skeleton->faces_begin(); f != skeleton->faces_end(); f++)
    {
        auto start = f->halfedge();
        auto current = start;
        std::vector<LXtPointID> points;
        do {
            // Get the target vertex of the half-edge
            LXtPointID pntID = vertex_to_pntID[current->vertex()];
            points.push_back(pntID);
            current = current->next();
        } while (current != start); // Repeat until end of the loop

        LXtPolygonID polyID;
        polygon.NewProto(LXiPTYP_FACE, points.data(), points.size(), false, &polyID);
        pols.push_back(polyID);
    }

    // Remove the source polygon
    polygon.Remove();

    return LXe_OK;
}

//
// Extrude the given polygon using CGAL::extrude_skeleton method.
//
LxResult CSkeleton::Extrude(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols, std::vector<LXtPolygonID>& sides)
{
    if (m_height== 0.0)
        return LXe_OK;

    CLxUser_Point point, point1;
    point.fromMesh(m_mesh);
    point1.fromMesh(m_mesh);

    LXtVector norm;
    polygon.Normal(norm);

    // Set axis plane to compute the triangulation on 2D space.
    AxisPlane axisPlane(norm);

    std::vector<LXtPointID> source;

    std::vector<std::vector<LXtPointID>> loops;
    MeshUtil::MakeBoundaryVertexList(m_mesh, polygon, source, loops);

    Polygon_with_holes_2 poly;

    double   z_ave = 0.0;
    for (auto i = 0u; i < source.size(); i++)
    {
        point.Select(source[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        poly.outer_boundary().push_back(Point_2(x, y));
        z_ave += z;
    }

    // averaged z value on axis plane
    z_ave /= static_cast<double>(source.size());

    for (auto& loop : loops)
    {
        Polygon_2 hole;
        for (auto i = 0u; i < loop.size(); i++)
        {
            point.Select(loop[i]);
            LXtFVector pos;
            point.Pos(pos);
            double x, y, z;
            axisPlane.ToPlane(pos, x, y, z);
            hole.push_back(Point_2(x, y));
        }
        poly.add_hole(hole);
    }

    SurfaceMesh mesh;
    CGAL::extrude_skeleton(poly, mesh, CGAL::parameters::maximum_height(m_height));

    std::vector<LXtPointID> vertices;
    vertices.resize(mesh.number_of_vertices());

    for (auto v : mesh.vertices())
    {
        LXtPointID pntID;
        LXtVector  pos;
        axisPlane.FromPlane(pos, mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z() * m_scale + z_ave);
        m_poledit.NewVertex(pos, &pntID);
        vertices[v] = pntID;
    }

    bool is_opened = MeshUtil::PolygonIsOpened(m_mesh, polygon);

    for (auto f : mesh.faces())
    {
        unsigned topCount = 0, btmCount = 0, total = 0;
        for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
        {
            if (mesh.point(v).z() == m_height)
                topCount++;
            else if (mesh.point(v).z() == 0.0)
                btmCount++;
            total++;
        }
        if ((is_opened == false) && (btmCount == total))
            continue;
        std::vector<LXtPointID> points;
        for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
        {
            points.push_back(vertices[v]);
        }

        LXtPolygonID polyID;
        polygon.NewProto(LXiPTYP_FACE, points.data(), points.size(), false, &polyID);
        if (topCount == total)
            pols.push_back(polyID);
        else if (btmCount != total)
            sides.push_back(polyID);
    }

    return LXe_OK;
}

//
// Offset the give polygon and create it as new polygon.
//
LxResult CSkeleton::Duplicate(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols)
{
    if (m_offset == 0.0)
        return LXe_OK;

    CLxUser_Point point, point1;
    point.fromMesh(m_mesh);
    point1.fromMesh(m_mesh);

    LXtVector norm;
    polygon.Normal(norm);

    // Set axis plane to compute the triangulation on 2D space.
    AxisPlane axisPlane(norm);

    std::vector<LXtPointID> source;

    std::vector<std::vector<LXtPointID>> loops;
    MeshUtil::MakeBoundaryVertexList(m_mesh, polygon, source, loops);

    Polygon_with_holes_2 poly;

    double   z_ave = 0.0;
    for (auto i = 0u; i < source.size(); i++)
    {
        point.Select(source[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        poly.outer_boundary().push_back(Point_2(x, y));
        z_ave += z;
    }

    // averaged z value on axis plane
    z_ave /= static_cast<double>(source.size());

    for (auto& loop : loops)
    {
        Polygon_2 hole;
        for (auto i = 0u; i < loop.size(); i++)
        {
            point.Select(loop[i]);
            LXtFVector pos;
            point.Pos(pos);
            double x, y, z;
            axisPlane.ToPlane(pos, x, y, z);
            hole.push_back(Point_2(x, y));
        }
        poly.add_hole(hole);
    }

    LXtID4       type;
    polygon.Type(&type);

    for (auto i = 0u; i < m_steps; i++)
    {
        PolygonWithHolesPtrVector offset_polygons;
        double f = static_cast<double>(i + 1) / static_cast<double>(m_steps);
        
        if (m_offset < 0.0)
            offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(m_offset * (-1) * f,poly);
        else
            offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(m_offset * f,poly);

        for (auto i = 0u; i < offset_polygons.size(); ++i)
        {
            const Polygon_with_holes_2& pwh = *offset_polygons[i];
            std::vector<LXtPointID> points;
            for (auto v = pwh.outer_boundary().vertices_begin(); v != pwh.outer_boundary().vertices_end(); v++)
            {
                LXtPointID pntID;
                LXtVector  pos;
                axisPlane.FromPlane(pos, v->x(), v->y(), z_ave + m_shift * f);
                m_poledit.NewVertex(pos, &pntID);
                points.push_back(pntID);
            }
            std::vector<std::vector<LXtPointID>> holes;
            for(auto h = pwh.holes_begin(); h != pwh.holes_end(); h++)
            {
                std::vector<LXtPointID> hole;
                for (auto v = h->vertices_begin(); v != h->vertices_end(); v++)
                {
                    LXtPointID pntID;
                    LXtVector  pos;
                    axisPlane.FromPlane(pos, v->x(), v->y(), z_ave + m_shift * f);
                    m_poledit.NewVertex(pos, &pntID);
                    hole.push_back(pntID);
                }
                holes.push_back(hole);
            }
            if (holes.size () > 0)
            {
                std::vector<LXtPointID> keyhole;
                MeshUtil::MakeKeyhole(m_mesh, axisPlane, points, holes, keyhole);
                points = keyhole;
            }
            LXtPolygonID polyID;
            polygon.NewProto(type, points.data(), points.size(), 0, &polyID);
            pols.push_back(polyID);
        }
    }

    return LXe_OK;
}

//
// Find the closest offset vector from the given point.
//
static VertexIterator GetClosetVertex(AxisPlane& axisPlane, CLxUser_Point& point, PolygonWithHolesPtrVector& offset_polygons)
{
    LXtFVector pos;
    point.Pos(pos);
    double x, y, z;
    axisPlane.ToPlane(pos, x, y, z);

    double min_dist = std::numeric_limits<double>::max();
    VertexIterator min_vertex;
    for (auto i = 0u; i < offset_polygons.size(); ++i)
    {
        const Polygon_with_holes_2& pwh = *offset_polygons[i];
        for (auto v = pwh.outer_boundary().vertices_begin(); v != pwh.outer_boundary().vertices_end(); v++)
        {
            double dist = (x - v->x()) * (x - v->x()) + (y - v->y()) * (y - v->y());
            if (dist < min_dist)
            {
                min_dist = dist;
                min_vertex = v;
            }
        }
        for(auto h = pwh.holes_begin(); h != pwh.holes_end(); h++)
        {
            for (auto v = h->vertices_begin(); v != h->vertices_end(); v++)
            {
                double dist = (x - v->x()) * (x - v->x()) + (y - v->y()) * (y - v->y());
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_vertex = v;
                }
            }
        }
    }
    return min_vertex;
}

//
// Merge co-located points of the given polygon.
//
static void MergePoints(CLxUser_Mesh& mesh, CLxUser_Polygon& polygon, AxisPlane& axisPlane, PolygonWithHolesPtrVector& offset_polygons, double z_ave)
{
    CLxUser_Point point;
    point.fromMesh(mesh);

    CLxUser_Polygon polygon1;
    polygon1.fromMesh(mesh);

    CLxUser_MeshService s_mesh;
    LXtMarkMode mark_select;
    mark_select = s_mesh.SetMode(LXsMARK_SELECT);

    std::unordered_map<VertexIterator,LXtPointID, VertexIteratorHash, VertexIteratorEqual> vertex_to_pntID;
    std::vector<LXtPointID> polygon_points;
    std::unordered_map<LXtPointID,LXtPointID> point_to_merge;
    std::unordered_set<LXtPolygonID> polygon_to_update;

    unsigned nvert;
    polygon.VertexCount(&nvert);

    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID pntID;
        polygon.VertexByIndex(i, &pntID);
        point.Select(pntID);
        VertexIterator v = GetClosetVertex(axisPlane, point, offset_polygons);
        if (vertex_to_pntID.find(v) != vertex_to_pntID.end())
        {
            LXtPointID pntID1 = vertex_to_pntID[v];
            if (polygon_points.size() == 0 || polygon_points.back() != pntID1)
                polygon_points.push_back(pntID1);
            point_to_merge.insert(std::make_pair(pntID, pntID1));
            unsigned npol;
            point.PolygonCount(&npol);
            for (auto j = 0; j < npol; j++)
            {
                LXtPolygonID polID1;
                point.PolygonByIndex(j, &polID1);
                polygon1.Select(polID1);
                if (polygon1.TestMarks(mark_select) == LXe_FALSE)
                    polygon_to_update.insert(polID1);
            }
        }
        else
        {
            LXtVector  pos;
            axisPlane.FromPlane(pos, v->x(), v->y(), z_ave);
            point.SetPos(pos);
            vertex_to_pntID.insert(std::make_pair(v, pntID));
            polygon_points.push_back(pntID);
        }
    }

    // Update vertex list of the polygon
    if (polygon_points.size() < nvert)
        polygon.SetVertexList(polygon_points.data(), polygon_points.size(), 0);

    for (auto polID : polygon_to_update)
    {
        polygon1.Select(polID);
        polygon_points.clear();
        polygon1.VertexCount(&nvert);
        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID pntID;
            polygon1.VertexByIndex(i, &pntID);
            auto pair = point_to_merge.find(pntID);
            if (pair != point_to_merge.end())
            {
                LXtPointID pntID1 = pair->second;
                if (polygon_points.size() == 0 || polygon_points.back() != pntID1)
                    polygon_points.push_back(pntID1);
            }
            else
                polygon_points.push_back(pntID);
        }
        polygon1.SetVertexList(polygon_points.data(), polygon_points.size(), 0);
    }

    for (const auto& pair : point_to_merge)
    {
        point.Select(pair.first);
        point.Remove();
    }
}

//
// Offset polygon using create_interior_skeleton_and_offset_polygons_with_holes_2 method
//
LxResult CSkeleton::Offset(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols)
{
    if (m_offset == 0.0)
        return LXe_OK;

    CLxUser_Point point, point1;
    point.fromMesh(m_mesh);
    point1.fromMesh(m_mesh);

    LXtVector norm;
    polygon.Normal(norm);

    // Set axis plane to compute the triangulation on 2D space.
    AxisPlane axisPlane(norm);

    std::vector<LXtPointID> source;

    std::vector<std::vector<LXtPointID>> loops;
    MeshUtil::MakeBoundaryVertexList(m_mesh, polygon, source, loops);

    Polygon_with_holes_2 poly;

    double   z_ave = 0.0;
    for (auto i = 0u; i < source.size(); i++)
    {
        point.Select(source[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        poly.outer_boundary().push_back(Point_2(x, y));
        z_ave += z;
    }

    // averaged z value on axis plane
    z_ave /= static_cast<double>(source.size());

    // setup holes
    for (auto& loop : loops)
    {
        Polygon_2 hole;
        for (auto i = 0u; i < loop.size(); i++)
        {
            point.Select(loop[i]);
            LXtFVector pos;
            point.Pos(pos);
            double x, y, z;
            axisPlane.ToPlane(pos, x, y, z);
            hole.push_back(Point_2(x, y));
        }
        poly.add_hole(hole);
    }

    // Create offset polygons using Straight-Skeleton algorithm
    PolygonWithHolesPtrVector offset_polygons;
    
    if (m_offset < 0.0)
        offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(m_offset * (-1),poly);
    else
        offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(m_offset,poly);

    // Merge co-located offset points and update linking side polygons
    if (m_merge)
    {
        MergePoints(m_mesh, polygon, axisPlane, offset_polygons, z_ave);
    }
    // Set new offset positions to the source points of the polygon
    else
    {
        unsigned nvert;
        polygon.VertexCount(&nvert);

        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID vrt;
            polygon.VertexByIndex(i, &vrt);
            point.Select(vrt);
            VertexIterator v = GetClosetVertex(axisPlane, point, offset_polygons);
            LXtVector  pos;
            axisPlane.FromPlane(pos, v->x(), v->y(), z_ave);
            point.SetPos(pos);
        }
    }

    pols.push_back(polygon.ID());

    return LXe_OK;
}