//
// Skeleton class using Straight-Skeleton.
//
#pragma once

#include <lxsdk/lx_log.hpp>
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxvmath.h>
#include <lxsdk/lxu_matrix.hpp>
#include <lxsdk/lxu_quaternion.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <vector>
#include <unordered_set>

#include "util.hpp"

enum EditMode : int
{
    Skeleton = 0,
    Extrude = 1,
    Duplicate = 2,
    Offset = 3,
};

struct CSkeleton
{
    CSkeleton()
    {
        m_offset = 0.0;
        m_shift  = 0.0;
        m_steps  = 1;
        m_height = 0.0;
        m_scale  = 1.0;
        m_merge  = 1;
    }
    void SetMesh(CLxUser_Mesh& edit_mesh, CLxUser_Mesh& base_mesh)
    {
        m_mesh.set(edit_mesh);
        s_mesh.CreatePolygonEdit(m_poledit);
        m_poledit.SetMesh(edit_mesh, base_mesh);
    }

    //
    // Subdivide polygons using straight-skeleton algorithm.
    //
    LxResult Skeleton(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols);

    //
    // Extrude polygon using straight-skeleton algorithm without crossing edges.
    //
    LxResult Extrude(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols, std::vector<LXtPolygonID>& sides);

    //
    // Create offset polygon using straight-skeleton algorithm.
    //
    LxResult Duplicate(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols);

    //
    // Offset polygon using straight-skeleton algorithm.
    //
    LxResult Offset(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& pols);

    CLxUser_Mesh        m_mesh;
    CLxUser_PolygonEdit m_poledit;
    CLxUser_LogService  s_log;
    CLxUser_MeshService s_mesh;

    double m_offset;     // Maximum edge length of triangle
    double m_shift;      // Shift value for the offset polygon
    int    m_steps;      // Number of step
    double m_height;     // Height of the roof
    double m_scale;      // Scale of the height
    int    m_merge;      // Merge the vertices for Offset
};


class CVisitor : public CLxImpl_AbstractVisitor
{
public:
    CLxUser_Point      m_vert;
    CLxUser_Polygon    m_poly;
    CLxUser_Mesh       base_mesh, edit_mesh;
    CLxUser_LogService s_log;
    CSkeleton          m_skeleton;
    int                m_mode;        // offset value
    bool               succeeded;

    std::vector<LXtPolygonID> top_polygons;     // result extruded polygons
    std::vector<LXtPolygonID> side_polygons;    // side polygons from extrude mode

    CVisitor()
    {
        succeeded = true;
        m_mode = Duplicate;
    }

    bool IsValid()
    {
        using namespace boost::geometry;
        typedef model::d2::point_xy<double> Point;
        typedef model::segment<Point> Segment;

        struct VertexPos
        {
            Point pnt;
            LXtPointID vrt;
            LXtFVector pos;
        };
    
        LXtVector norm;
        m_poly.Normal(norm);

        AxisPlane axisPlane(norm);
    
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        std::vector<VertexPos> points;
    
        // Cache the vertex positions and points on the axis plane.
        for (auto i = 0u; i < nvert; i++)
        {
            VertexPos vp;
            m_poly.VertexByIndex(i, &vp.vrt);
            m_vert.Select(vp.vrt);
            m_vert.Pos(vp.pos);
            double x, y, z;
            axisPlane.ToPlane(vp.pos, x, y, z);
            vp.pnt = Point(x, y);
            points.push_back(vp);
        }

        // Check the polygon is convex or not. If the polygon is convex,
        // the polygon is not twisted.
        bool convex = true;
        for (auto i = 0u; i < nvert; i++)
        {
            auto j = (i + 1) % nvert;
            auto k = (i - 1 + nvert) % nvert;
    
            LXtVector a, b, c;

            LXx_VSUB3 (a, points[k].pos, points[i].pos);
            LXx_VSUB3 (b, points[i].pos, points[j].pos);
            LXx_VCROSS (c, a, b);
            if (LXx_VDOT (c, norm) < 0.0)
            {
                convex = false;
                break;
            }
        }
        if (convex)
            return true;

        // Concave quad polygon should be triangulated by Quadrangles method.
        if (nvert == 4)
            return false;
        
        // Check the polygon is twisted or not by checking the intersection
        for (auto i = 0u; i < nvert; i++)
        {
            auto j = (i + 1) % nvert;
            Segment s1(points[i].pnt, points[j].pnt);
            for (auto k = 0u; k < nvert; k++)
            {
                auto l = (k + 1) % nvert;
                if (i == k || i == l || j == k || j == l)
                    continue;
                if (points[i].vrt == points[k].vrt || points[i].vrt == points[l].vrt ||
                    points[j].vrt == points[k].vrt || points[j].vrt == points[l].vrt)
                    continue;
                Segment s2(points[k].pnt, points[l].pnt);
                if (intersects(s1, s2))
                {
                    s_log.DebugOut(LXi_DBLOG_NORMAL, "polygon ID %p is twisted", m_poly.ID());
                    return false;
                }
            }
        }
        return true;
    }

    LxResult Evaluate()
    {
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        if (nvert < 3)
            return LXe_OK;

        LXtID4 type;
        m_poly.Type(&type);
        if ((type != LXiPTYP_FACE) && (type != LXiPTYP_PSUB) && (type != LXiPTYP_SUBD))
            return LXe_OK;
    
        // the polygon is valid or not.
        bool is_valid = IsValid();
        if (!is_valid)
        {
            succeeded = false;
            return LXe_OK;
        }

        LxResult result = LXe_OK;

        if (m_mode == Skeleton)
            result = m_skeleton.Skeleton(m_poly, top_polygons);
        else if (m_mode == Extrude)
            result = m_skeleton.Extrude(m_poly, top_polygons, side_polygons);
        else if (m_mode == Duplicate)
            result = m_skeleton.Duplicate(m_poly, top_polygons);
        else if (m_mode == Offset)
            result = m_skeleton.Offset(m_poly, top_polygons);
    
        // check the result
        if (result != LXe_OK)
            succeeded = false;
    
        return result;
    }
};
