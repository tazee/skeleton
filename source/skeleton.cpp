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

//    int i = 0;
    for (auto v = skeleton->vertices_begin(); v != skeleton->vertices_end(); v++)
    {
        LXtPointID pntID;
        LXtVector  pos;
        axisPlane.FromPlane(pos, v->point().x(), v->point().y(), z_ave);
//        printf("[%d] x = %f, y = %f contour (%d) skeleton (%d) split (%d) vert.new %f %f %f\n", 
//            i++, v->point().x(), v->point().y(), v->is_contour(), v->is_skeleton(), v->is_split(), pos[0], pos[1], pos[2]);
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


static bool MergeLoops(std::vector<LXtPointID>& target, std::vector<LXtPointID>& loop)
{
    for (auto i = 0u; i < target.size(); i++)
    {
        auto i0 = (i + 1) % target.size();
        for (auto j = 0u; j < loop.size(); j++)
        {
            if ((target[i] == loop[(j + 1) % loop.size()]) && (target[i0] == loop[j]))
            {
                auto j0 = (j + 2) % loop.size();
                auto n0 = loop.size() - 2;
                std::vector<LXtPointID> connect;
                connect.resize(n0);
                for (auto k = 0; k < n0; k++)
                    connect[k] = loop[(j0 + k) % loop.size()];
                target.insert(target.begin() + i0, connect.begin(), connect.end());
                return true;
            }
        }
    }
    return false;
}


static void FixStartPoint(CLxUser_Mesh& mesh, CLxUser_Polygon& polygon, std::vector<LXtPointID>& points)
{
    LXtVector norm;
    polygon.Normal(norm);
    CLxVector cnorm(norm);
    std::vector<CLxVector> vecs;

    CLxUser_Point point;
    point.fromMesh(mesh);
    for (auto i = 0u; i < points.size(); i++)
    {
        point.Select(points[i]);
        LXtFVector pos;
        point.Pos(pos);
        vecs.push_back(CLxVector(pos));
    }
    for (auto i = 0u; i < vecs.size(); i++)
    {
        CLxVector v0 = vecs[(i + 1) % vecs.size()] - vecs[i];
        CLxVector v1 = vecs[(i + vecs.size() - 1) % vecs.size()] - vecs[i];
        CLxVector cross = v0.cross(v1);
        if (cross.dot(cnorm) > 0.0)
        {
            if (i != 0)
            {
                std::vector<LXtPointID> points1;
                for (auto j = 0u; j < points.size(); j++)
                {
                    points1.push_back(points[(j + i) % points.size()]);
                }
                points.clear();
                points = points1;
            }
            return;
        }
    }
}


static void MergePolygons(CLxUser_Mesh& mesh, CLxUser_Polygon& polygon, std::vector<std::vector<LXtPointID>>& top_polys, std::vector<LXtPolygonID>& pols)
{
    printf("MergePolygons = %lu\n", top_polys.size());
    while (top_polys.size() > 0)
    {
        std::vector<LXtPointID> points(top_polys[0].begin(), top_polys[0].end());
        top_polys.erase(top_polys.begin());
        printf("** pop points = %lu top_polys = %lu\n", points.size(), top_polys.size());

        while (top_polys.size() > 0)
        {
            bool connected = false;
            for (auto i = 0u; i < top_polys.size(); i++)
            {
                if (MergeLoops(points, top_polys[i]))
                {
                    top_polys.erase(top_polys.begin() + i);
                    connected = true;
                }
            }
            if (connected == false)
            {
#if 0
                printf("-- no connect top_polys = %zu\n", top_polys.size());
                for (auto j = 0u; j < points.size(); j++)
                    printf("[%u] target point %p\n", j, points[j]);
                for (auto i = 0u; i < top_polys.size(); i++)
                {
                    printf("[%u] top_polys = %zu\n", i, top_polys[i].size());
                    for (auto j = 0u; j < top_polys[i].size(); j++)
                        printf("\t[%u] loop point %p\n", j, top_polys[i][j]);
                }
#endif
                break;
            }
        }

        if (points.size() > 2)
        {
            LXtPolygonID polyID;
            FixStartPoint(mesh, polygon, points);
            polygon.NewProto(LXiPTYP_FACE, points.data(), points.size(), false, &polyID);
            printf("NewPolygon %p nvert = %zu\n", polyID, points.size());
            pols.push_back(polyID);
        }
    }
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

    printf("mesh vertices = %lu\n", mesh.vertices().size());
    for (auto v : mesh.vertices())
    {
        LXtPointID pntID;
        LXtVector  pos;
        axisPlane.FromPlane(pos, mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z() * m_scale + z_ave);
        m_poledit.NewVertex(pos, &pntID);
        vertices[v] = pntID;
        std::cout << "v:" << v << ")\n";
        printf("point %f %f %f\n", mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z());

#if 0
        auto halfedge = mesh.halfedge(v); // 頂点に関連付けられたハーフエッジを取得
        CGAL::Halfedge_around_target_circulator<SurfaceMesh> circulator(halfedge, mesh), done(circulator);
        do {
            // ハーフエッジ情報を出力
            auto s = mesh.source(*circulator); // 始点の頂点
            auto t = mesh.source(*circulator);

            std::cout << "s:" << s << "t:" << t << "\n";
            printf("halfedge source %f %f %f target %f %f %f\n", 
                mesh.point(s).x(), mesh.point(s).y(), mesh.point(s).z(),
                mesh.point(t).x(), mesh.point(t).y(), mesh.point(t).z());
            ++circulator; // 次のハーフエッジへ移動
        } while (circulator != done); // 循環終了条件
#endif
    }

    bool is_opened = MeshUtil::PolygonIsOpened(m_mesh, polygon);
    bool merge_top = true;
    std::vector<std::vector<LXtPointID>> top_polys;

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
        if (btmCount == total)
            continue;
        std::vector<LXtPointID> points;
        for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
        {
            points.push_back(vertices[v]);
        }

        if (merge_top && (topCount == total))
        {
            top_polys.push_back(points);
            continue;
        }

        LXtPolygonID polyID;
        polygon.NewProto(LXiPTYP_FACE, points.data(), points.size(), false, &polyID);
        if (topCount == total)
            pols.push_back(polyID);
        else if (btmCount != total)
            sides.push_back(polyID);
    }

    if (merge_top)
    {
        MergePolygons(m_mesh, polygon, top_polys, pols);
    }

    if (is_opened == false)
        polygon.Remove();
    else
    {
        source.clear();
        unsigned nvert;
        polygon.VertexCount(&nvert);
        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID vrt;
            polygon.VertexByIndex(i, &vrt);
            source.push_back(vrt);
        }
        polygon.SetVertexList(source.data(), source.size(), 1);
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


static void SplitInsetVertices(std::vector<LXtPointID>& vertices, 
    std::vector<LXtPointID>& source, std::vector<std::vector<LXtPointID>>& loops, 
    std::vector<LXtPointID>& inset_outer, std::vector<std::vector<LXtPointID>>& inset_loops)
{
    inset_outer.clear();
    inset_loops.clear();
    for (auto i = 0; i < source.size(); i++)
    {
        inset_outer.push_back(vertices[i]);
    }
    auto offset = source.size();
    for (auto& loop : loops)
    {
        std::vector<LXtPointID> inset_loop;
        for (auto i = 0; i < loop.size(); i++)
        {
            inset_loop.push_back(vertices[i + offset]);
        }
        inset_loops.push_back(inset_loop);
        offset += loop.size();
    }
}

//
// Inset the given polygon using CGAL::extrude_skeleton method.
//
LxResult CSkeleton::Inset(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols, std::vector<LXtPolygonID>& sides)
{
    if (m_offset <= 0.0)
        return LXe_OK;

    printf("** Inset start merge = %d offset = %f\n", m_merge, m_offset);
    CLxUser_Point point, point1;
    point.fromMesh(m_mesh);
    point1.fromMesh(m_mesh);

    LXtVector norm;
    polygon.Normal(norm);

    // Set axis plane to compute the triangulation on 2D space.
    AxisPlane axisPlane(norm);

    // Get the source points of the polygon and split it into the outer boundary and holes
    std::vector<LXtPointID> source;
    std::vector<std::vector<LXtPointID>> loops;
    MeshUtil::MakeBoundaryVertexList(m_mesh, polygon, source, loops);

    Polygon_with_holes_2 poly;

    // Create a polygon with holes on 2D space for CGAL algorithm
    auto total_points = source.size();
    double z_ave = 0.0;

    for (auto i = 0u; i < source.size(); i++)
    {
        point.Select(source[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        poly.outer_boundary().push_back(Point_2(x, y));
        z_ave += z;
        //printf("[%u] source %f %f %f vert.new %f %f %f\n", i, x, y, z, pos[0], pos[1], pos[2]);
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
        total_points += loop.size();
    }

    // Create a straight skeleton and subdivide the polygon
    SurfaceMesh mesh;
    CGAL::extrude_skeleton(poly, mesh, CGAL::parameters::maximum_height(m_offset));

    // All vertices of the inset polygon
    std::vector<LXtPointID> vertices;

    // Set the search tolerance for the co-located vertices to merge
    if (m_merge)
    {
        double tol = lx::Tolerance(m_offset);
        m_poledit.SetSearch(1, tol);
    }

    // Store skeleton vertices for the source points of the polygon
    std::vector<CGAL::SM_Vertex_index> sm_source_verices;

    double z_max = 0.0;
    for (auto v : mesh.vertices())
    {
        if (lx::Compare(mesh.point(v).z(), z_max) > 0)
            z_max = mesh.point(v).z();
        if (sm_source_verices.size() < total_points)
            sm_source_verices.push_back(v);
    }

    printf("** Inset start z_max %f sm_source_verices = %zu\n", z_max, sm_source_verices.size());
    for (auto v : sm_source_verices)
    {
        LXtPointID pntID;
        LXtVector pos1;

        // target vertex started from source point of the polygon
        // all tracing vertices are stored in trace_list
        auto vt = v;
        std::vector<CGAL::SM_Vertex_index> trace_list(vt);
    
        while (lx::Compare(mesh.point(vt).z(), z_max) < 0)
        {
            bool updated = false;

            auto halfedge = mesh.halfedge(vt); // Get halfedge associated to the given vertex
            CGAL::Halfedge_around_target_circulator<SurfaceMesh> circulator(halfedge, mesh), done(circulator);

            auto v0 = vt;   // the beginning of the branch
            std::vector<CGAL::SM_Vertex_index> source_list;
            double dist = std::numeric_limits<double>::max();
    
            // Find the closest vertex to the target vertex from the begging of the branch
            do {
                // Get the target vertex of the half-edge
                auto s = mesh.source(*circulator); // source vertex
                source_list.push_back(s);

                // Compute the distance between the source vertex and the target vertex
                int comp = lx::Compare(mesh.point(s).z(), mesh.point(v0).z());
                double x = mesh.point(v0).x() - mesh.point(s).x();
                double y = mesh.point(v0).y() - mesh.point(s).y();
                double dist1 = x * x + y * y;

                // Update the target vertex if the source vertex is closer to the beginning of the branch
                if (comp > 0)
                {
                    int comp_z = lx::Compare(mesh.point(s).z(), mesh.point(vt).z());
                    int comp_dist = lx::Compare(dist1, dist);
                    if ((comp_dist < 0) || ((comp_dist = 0) && (comp_z > 0)))
                    {
                        vt = s;
                        dist = dist1;
                        updated = true;
                    }
                }
                ++circulator; // Move to next halfedge
            } while (circulator != done); // iterate until the end of the loop

            // Find the co-located vertex at the beginning of the branch
            if (updated == false)
            {
                for (auto s : source_list)
                {
                    if (std::find(trace_list.begin(), trace_list.end(), s) == trace_list.end())
                    {
                        double x = mesh.point(v0).x() - mesh.point(s).x();
                        double y = mesh.point(v0).y() - mesh.point(s).y();
                        double z = mesh.point(v0).z() - mesh.point(s).z();
                        double dist1 = x * x + y * y + z * z;
                        if (lx::Compare(dist1, 0.0) == 0)
                        {
                            vt = s;
                            updated = true;
                            break;
                        }
                    }
                }
            }

            if (updated == false)
                break;
            
            trace_list.push_back(vt);
        }

        axisPlane.FromPlane(pos1, mesh.point(vt).x(), mesh.point(vt).y(), z_ave);
        m_poledit.NewVertex(pos1, &pntID);
        vertices.push_back(pntID);
    }

    // Split the inset vertices into the outer vertices and the loop vertices
    std::vector<LXtPointID> inset_outer;
    std::vector<std::vector<LXtPointID>> inset_loops;
    SplitInsetVertices(vertices, source, loops, inset_outer, inset_loops);

    LXtID4       type;
    polygon.Type(&type);

    LXtPolygonID polyID;
    std::vector<LXtPointID> outer;

    for (auto i = 0u; i < inset_outer.size(); i++)
    {
        if ((i == 0) || (inset_outer[i] != inset_outer[i - 1] && inset_outer[i] != inset_outer[0]))
            outer.push_back(inset_outer[i]);
    }

    for (auto i = 0u; i < source.size(); i++)
    {
        auto j = (i + 1) % source.size();
        std::vector<LXtPointID> quads;
        quads.push_back(source[i]);
        quads.push_back(source[j]);
        quads.push_back(inset_outer[j]);
        if (inset_outer[i] != inset_outer[j])
            quads.push_back(inset_outer[i]);
        LXtPolygonID polyID;
        polygon.NewProto(type, quads.data(), quads.size(), 0, &polyID);
        sides.push_back(polyID);
    }

    printf("InsetPolygon vertices = %zu outer = %zu %zu loops = %zu\n", vertices.size(), inset_outer.size(), outer.size(), inset_loops.size());
    if (inset_loops.size() == 0)
    {
        if (outer.size() > 2)
        {
            printf("** top polygons = %zu\n", outer.size());
        //    FixStartPoint(m_mesh, polygon, outer);
            polygon.NewProto(type, outer.data(), outer.size(), 0, &polyID);
            pols.push_back(polyID);
        }
    }
    else
    {
        std::vector<std::vector<LXtPointID>> holes;
        for (auto i = 0u; i < inset_loops.size(); i++)
        {
            auto& inset_loop = inset_loops[i];
            auto& loop = loops[i];
        
            std::vector<LXtPointID> hole;
            for (auto i = 0u; i < inset_loop.size(); i++)
            {
                if ((i == 0) || (inset_loop[i] != inset_loop[i - 1] && inset_loop[i] != inset_loop[0]))
                    hole.push_back(inset_loop[i]);
            }
            holes.push_back(hole);

            for (auto i = 0u; i < loop.size(); i++)
            {
                auto j = (i + 1) % loop.size();
                std::vector<LXtPointID> quads;
                quads.push_back(loop[i]);
                quads.push_back(loop[j]);
                quads.push_back(inset_loop[j]);
                if (inset_loop[i] != inset_loop[j])
                    quads.push_back(inset_loop[i]);
                LXtPolygonID polyID;
                polygon.NewProto(type, quads.data(), quads.size(), 0, &polyID);
                sides.push_back(polyID);
            }
        }
        std::vector<LXtPointID> keyhole;
        MeshUtil::MakeKeyhole(m_mesh, axisPlane, outer, holes, keyhole);
        polygon.NewProto(type, keyhole.data(), keyhole.size(), 0, &polyID);
        pols.push_back(polyID);
    }

    polygon.Remove();

    return LXe_OK;
}
