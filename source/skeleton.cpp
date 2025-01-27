//
// Extrude class to wrap CGAL Traingulation library.
//
#include "lxsdk/lxresult.h"
#include "lxsdk/lxvmath.h"
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxu_matrix.hpp>
#include <lxsdk/lxu_quaternion.hpp>
#include <lxsdk/lxu_geometry_triangulation.hpp>

#include <vector>
#include <map>
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
//#include <CGAL/Gps_traits_2.h>

#include "skeleton.hpp"


//
// Return true if the vertex pair is appeared twice.
//
static bool IsKeyholeBridge(CLxUser_Point& point, CLxUser_Point& point1)
{
    CLxUser_MeshService s_mesh;
    LXtMarkMode mark_dupl = s_mesh.SetMode(LXsMARK_USER_0);

    if (point.TestMarks(mark_dupl) == LXe_FALSE)
        return false;
    if (point1.TestMarks(mark_dupl) == LXe_FALSE)
        return false;
    return true;
}

//
// Make vertex index table of polygon.
//
static void MakeVertexTable(CLxUser_Polygon& polygon, CLxUser_Point& point, std::vector<LXtPointID>& vertices, std::unordered_map<LXtPointID,unsigned>& indices)
{
    CLxUser_MeshService s_mesh;

    LXtMarkMode mark_dupl;
    mark_dupl = s_mesh.ClearMode(LXsMARK_USER_0);

    unsigned nvert;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        polygon.VertexByIndex(i, &vrt);
        point.Select(vrt);
        point.SetMarks(mark_dupl);
    }

    mark_dupl = s_mesh.SetMode(LXsMARK_USER_0);

    unsigned n = 0;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        polygon.VertexByIndex(i, &vrt);
        if (indices.find(vrt) == indices.end())
        {
            indices.insert(std::make_pair(vrt, n ++));
            vertices.push_back(vrt);
        }
        else
        {
            point.Select(vrt);
            point.SetMarks(mark_dupl);
        }
    }
}


//
// Compute the area of the loop on axis plane.
//
static double LoopAreaSize(CLxUser_Mesh& mesh, AxisPlane& axisPlane, std::vector<LXtPointID>& loop)
{
    if (loop.size() < 3)
        return 0.0;

    CLxUser_Point point;
    point.fromMesh(mesh);

    LXtFVector pos;
    double x0, y0, z0, x1, y1, z1;
    point.Select(loop[0]);
    point.Pos(pos);
    axisPlane.ToPlane(pos, x0, y0, z0);
    point.Select(loop[1]);
    point.Pos(pos);
    axisPlane.ToPlane(pos, x1, y1, z1);
    double ax, ay, bx, by;
    double S = 0.0;
    ax = x1 - x0;
    ay = y1 - y0;
    for (auto i = 2u; i < loop.size(); i++)
    {
        point.Select(loop[i]);
        point.Pos(pos);
        axisPlane.ToPlane(pos, x1, y1, z1);
        bx = x1 - x0;
        by = y1 - y0;
		S += (ax * by - ay * bx) * 0.5;
		ax = bx;
		ay = by;
    }
    return std::abs(S);
}

//
// Make boundary vertex list and loops from the given polygon.
//
static bool MakeBoundaryVertexList(CLxUser_Mesh& mesh, CLxUser_Polygon& polygon, std::vector<LXtPointID>& vertices, std::vector<std::vector<LXtPointID>>& loops)
{
    CLxUser_Point point0, point1;
    point0.fromMesh(mesh);
    point1.fromMesh(mesh);

    vertices.clear();
    loops.clear();

    bool hasHole = false;

    std::vector<std::pair<LXtPointID,LXtPointID>> edges;

    unsigned nvert;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        auto j = nvert - i - 1;
        LXtPointID vrt0, vrt1;
        polygon.VertexByIndex(j, &vrt0);
        polygon.VertexByIndex((j + 1) % nvert, &vrt1);
        point0.Select(vrt0);
        point1.Select(vrt1);
        if (IsKeyholeBridge(point0, point1))
        {
            hasHole = true;
        }
        else
        {
            edges.push_back(std::make_pair(vrt0, vrt1));
        }
    }

printf("-- hasHole = %d\n", hasHole);
    if (!hasHole)
    {
        vertices.resize(nvert);
        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID vrt;
            polygon.VertexByIndex(i, &vrt);
            vertices[i] = vrt;
        }
        return false;
    }

    // split the polygon into loops
    std::vector<LXtPointID> loop;
    std::pair<LXtPointID,LXtPointID> edge;

    while (edges.size() > 0)
    {
        if (loop.empty())
        {
            edge = edges.back();
            edges.pop_back();
            loop.push_back(edge.first);
            printf("start edge (%p, %p)\n", edge.first, edge.second);
        }

        bool found = false;
        for(auto it = edges.begin(); it != edges.end(); it++)
        {
            if (it->first == edge.second)
            {
                loop.push_back(it->first);
                printf("next edge (%p, %p) loop = %zu [%p]\n", it->first, it->second, loop.size(), loop[0]);
                edge = *it;
                if (it->second == loop[0])
                {
                    loops.push_back(loop);
                    loop.clear();
                }
                edges.erase(it);
                found = true;
                break;
            }
        }
        if (!found)
        {
            vertices.resize(nvert);
            for (auto i = 0u; i < nvert; i++)
            {
                LXtPointID vrt;
                polygon.VertexByIndex(i, &vrt);
                vertices[i] = vrt;
            }
            return false;
        }
    }

    // find the outer loop
    LXtVector norm;
    polygon.Normal(norm);

    AxisPlane axisPlane(norm);

    printf("MakeBoundaryVertexList: loops.size() = %lu\n", loops.size());
    double max_area = LoopAreaSize (mesh, axisPlane, loops[0]);
    unsigned index = 0;
    printf("loops[0].size() = %lu max_area %f\n", loops[0].size(), max_area);

    for (auto i = 1u; i < loops.size(); i++)
    {
        double area = LoopAreaSize(mesh, axisPlane, loops[i]);
    printf("loops[%d].size() = %lu max_area %f\n", i, loops[i].size(), area);
        if (area > max_area)
        {
            max_area = area;
            index = i;
        }
    }

    vertices = loops[index];
    loops.erase(loops.begin() + index);

    return true;
}

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

typedef std::shared_ptr<Polygon_with_holes_2> PolygonWithHolesPtr ;
typedef std::vector<PolygonWithHolesPtr> PolygonWithHolesPtrVector;

//typedef CGAL::Gps_traits_2<K>           Traits;

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
    MakeBoundaryVertexList(m_mesh, polygon, source, loops);

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

    printf("skeleton->size_of_vertices() = %lu\n", skeleton->size_of_vertices());
    int i = 0;
    for (auto v = skeleton->vertices_begin(); v != skeleton->vertices_end(); v++)
    {
        LXtPointID pntID;
        LXtVector  pos;
        printf("[%d] x = %f, y = %f contour (%d) skeleton (%d) split (%d)\n", 
            i++, v->point().x(), v->point().y(), v->is_contour(), v->is_skeleton(), v->is_split());
        axisPlane.FromPlane(pos, v->point().x(), v->point().y(), z_ave);
        if (v->id() < source.size())
            pntID = source[v->id()];
        else
            m_poledit.AddFaceVertex(pos, polygon.ID(), nullptr, &pntID);
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

LxResult CSkeleton::Extrude(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols)
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
    std::unordered_map<LXtPointID,unsigned> indices;

    MakeVertexTable(polygon, point, source, indices);

    printf("**Extrude m_height = %f\n", m_height);
    std::vector<std::vector<LXtPointID>> loops;
    MakeBoundaryVertexList(m_mesh, polygon, source, loops);

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

    for (auto f : mesh.faces())
    {
        std::vector<LXtPointID> points;
        for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
        {
            points.push_back(vertices[v]);
        }

        LXtPolygonID polyID;
        polygon.NewProto(LXiPTYP_FACE, points.data(), points.size(), false, &polyID);
        pols.push_back(polyID);
    }

    return LXe_OK;
}

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
#if 0
    std::unordered_map<LXtPointID,unsigned> indices;

    MakeVertexTable(polygon, point, source, indices);

    Polygon_2 poly;
#endif

    std::vector<std::vector<LXtPointID>> loops;
    MakeBoundaryVertexList(m_mesh, polygon, source, loops);

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

        printf("inset_polygons.size() = %lu\n", offset_polygons.size());
        bool first = (m_offset < 0.0) ? true : false;
        for (auto i = 0u; i < offset_polygons.size(); ++i)
        {
            const Polygon_with_holes_2& pwh = *offset_polygons[i];
            if (first) {
                first = false;
            //    continue;
            }
            std::vector<LXtPointID> points;
            for (auto v = pwh.outer_boundary().vertices_begin(); v != pwh.outer_boundary().vertices_end(); v++)
            {
                LXtPointID pntID;
                LXtVector  pos;
                axisPlane.FromPlane(pos, v->x(), v->y(), z_ave + m_shift * f);
                m_poledit.NewVertex(pos, &pntID);
                points.push_back(pntID);
            }

            LXtPolygonID polyID;
            polygon.NewProto(type, points.data(), points.size(), 0, &polyID);
            pols.push_back(polyID);
        }
    }

    return LXe_OK;
}