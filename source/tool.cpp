/*
 * TOOL.CPP  Skeleton tool and tool operation using straight-skeleton algorithm. 
 */

#include "tool.hpp"

/*
 * On create we add our one tool attribute. We also allocate a vector type
 * and select mode mask.
 */
CTool::CTool()
{
    static const LXtTextValueHint skeleton_mode[] = {
        { Skeleton, "skeleton" }, 
        { Extrude, "extrude" }, 
        { Duplicate, "duplicate" }, 
        { 0, "=skeleton_mode" }, 0
    };

    CLxUser_PacketService sPkt;
    CLxUser_MeshService   sMesh;

    dyna_Add(ATTRs_MODE, LXsTYPE_INTEGER);
    dyna_SetHint(ATTRa_MODE, skeleton_mode);

    dyna_Add(ATTRs_OFFSET, LXsTYPE_DISTANCE);

    dyna_Add(ATTRs_SHIFT, LXsTYPE_DISTANCE);

    dyna_Add(ATTRs_STEPS, LXsTYPE_INTEGER);

    dyna_Add(ATTRs_HEIGHT, LXsTYPE_DISTANCE);

    dyna_Add(ATTRs_SCALE, LXsTYPE_PERCENT);

    tool_Reset();

    sPkt.NewVectorType(LXsCATEGORY_TOOL, v_type);
    sPkt.AddPacket(v_type, LXsP_TOOL_VIEW_EVENT, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_SCREEN_EVENT, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_FALLOFF, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_SUBJECT2, LXfVT_GET);

    offset_view = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_VIEW_EVENT);
    offset_screen = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SCREEN_EVENT);
    offset_falloff = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_FALLOFF);
    offset_subject = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SUBJECT2);
    mode_select = sMesh.SetMode("select");
}

/*
 * Reset sets the attributes back to defaults.
 */
void CTool::tool_Reset()
{
    CSkeleton ss;
    dyna_Value(ATTRa_MODE).SetInt(Skeleton);
    dyna_Value(ATTRa_OFFSET).SetFlt(ss.m_offset);
    dyna_Value(ATTRa_SHIFT).SetFlt(ss.m_shift);
    dyna_Value(ATTRa_STEPS).SetInt(ss.m_steps);
    dyna_Value(ATTRa_HEIGHT).SetFlt(ss.m_height);
    dyna_Value(ATTRa_SCALE).SetFlt(ss.m_scale);
}

/*
 * Boilerplate methods that identify this as an action (state altering) tool.
 */
LXtObjectID CTool::tool_VectorType()
{
    return v_type.m_loc;  // peek method; does not add-ref
}

const char* CTool::tool_Order()
{
    return LXs_ORD_ACTR;
}

LXtID4 CTool::tool_Task()
{
    return LXi_TASK_ACTR;
}

LxResult CTool::tool_GetOp(void** ppvObj, unsigned flags)
{
    CLxSpawner<CToolOp> spawner(SRVNAME_TOOLOP);
    CToolOp*            toolop = spawner.Alloc(ppvObj);

	if (!toolop)
	{
		return LXe_FAILED;
	}

    dyna_Value(ATTRa_MODE).GetInt(&toolop->m_mode);
    dyna_Value(ATTRa_OFFSET).GetFlt(&toolop->m_offset);
    dyna_Value(ATTRa_SHIFT).GetFlt(&toolop->m_shift);
    dyna_Value(ATTRa_STEPS).GetInt(&toolop->m_steps);
    dyna_Value(ATTRa_HEIGHT).GetFlt(&toolop->m_height);
    dyna_Value(ATTRa_SCALE).GetFlt(&toolop->m_scale);

    toolop->offset_view = offset_view;
    toolop->offset_screen = offset_screen;
    toolop->offset_falloff = offset_falloff;
    toolop->offset_subject = offset_subject;

	return LXe_OK;
}

LXtTagInfoDesc CTool::descInfo[] =
{
	{LXsTOOL_PMODEL, "."},
	{LXsTOOL_USETOOLOP, "."},
	{LXsPMODEL_SELECTIONTYPES, LXsSELOP_TYPE_POLYGON},
	{0}

};

/*
 * We employ the simplest possible tool model -- default hauling. We indicate
 * that we want to haul one attribute, we name the attribute, and we implement
 * Initialize() which is what to do when the tool activates or re-activates.
 * In this case set the axis to the current value.
 */
unsigned CTool::tmod_Flags()
{
    return LXfTMOD_I0_INPUT;
}

LxResult CTool::tmod_Enable(ILxUnknownID obj)
{
    CLxUser_Message msg(obj);

    if (TestPolygon() == false)
    {
        msg.SetCode(LXe_CMD_DISABLED);
        msg.SetMessage(SRVNAME_TOOL, "NoPolygon", 0);
        return LXe_DISABLED;
    }
    return LXe_OK;
}

LxResult CTool::tmod_Down(ILxUnknownID vts, ILxUnknownID adjust)
{
    dyna_Value(ATTRa_OFFSET).GetFlt(&m_offset);
    return LXe_TRUE;
}

void CTool::tmod_Move(ILxUnknownID vts, ILxUnknownID adjust)
{
    CLxUser_AdjustTool at(adjust);
    CLxUser_VectorStack vec(vts);
    LXpToolScreenEvent* spak = static_cast<LXpToolScreenEvent*>(vec.Read(offset_screen));

    double offset = m_offset0 + (spak->cx - spak->px) * 0.1;
    if (offset < 0.0)
        offset = 0.0;
    at.SetFlt(ATTRa_OFFSET, offset);
}

void CTool::atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints)
{
    switch (index)
    {
        case ATTRa_SHIFT:
        case ATTRa_HEIGHT:
        case ATTRa_SCALE:
            hints.MinFloat(0.0);
            break;
        
        case ATTRa_STEPS:
            hints.MinInt(1);
            break;
    }
}

bool CTool::TestPolygon()
{
    /*
     * Start the scan in read-only mode.
     */
    CLxUser_LayerScan scan;
    CLxUser_Mesh      mesh;
    unsigned          i, n, count;
    bool              ok = false;

    s_layer.BeginScan(LXf_LAYERSCAN_ACTIVE | LXf_LAYERSCAN_MARKPOLYS, scan);

    /*
     * Count the polygons in all mesh layers.
     */
    if (scan)
    {
        n = scan.NumLayers();
        for (i = 0; i < n; i++)
        {
            scan.BaseMeshByIndex(i, mesh);
            mesh.PolygonCount(&count);
            if (count > 0)
            {
                ok = true;
                break;
            }
        }
        scan.Apply();
    }

    /*
     * Return false if there is no polygons in any active layers.
     */
    return ok;
}

/*
 * Tool evaluation uses layer scan interface to walk through all the active
 * meshes and visit all the selected polygons.
 */
LxResult CToolOp::top_Evaluate(ILxUnknownID vts)
{
    printf("top_Evaluate: offset = %f\n", m_offset);
    CLxUser_VectorStack vec(vts);

    /*
     * Start the scan in edit mode.
     */
    CLxUser_LayerScan  scan;
    CLxUser_Mesh       base_mesh, edit_mesh;

    if (vec.ReadObject(offset_subject, subject) == false)
        return LXe_FAILED;
    if (vec.ReadObject(offset_falloff, falloff) == false)
        return LXe_FAILED;

    CLxUser_MeshService   s_mesh;

    LXtMarkMode pick = s_mesh.SetMode(LXsMARK_SELECT);

    subject.BeginScan(LXf_LAYERSCAN_EDIT_POLYS, scan);

    auto n = scan.NumLayers();
    for (auto i = 0u; i < n; i++)
    {
        CVisitor vis;
    
        scan.BaseMeshByIndex(i, vis.base_mesh);
        scan.EditMeshByIndex(i, vis.edit_mesh);
    
        vis.m_skeleton.SetMesh(vis.edit_mesh, vis.base_mesh);
        vis.m_mode = m_mode;
        vis.m_skeleton.m_offset = m_offset;
        vis.m_skeleton.m_shift  = m_shift;
        vis.m_skeleton.m_steps  = m_steps;
        vis.m_skeleton.m_height = m_height;
        vis.m_skeleton.m_scale  = m_scale;

        check(vis.m_vert.fromMesh(vis.edit_mesh));
        check(vis.m_poly.fromMesh(vis.edit_mesh));

        // Triangulate selected polygons using CDT traingulation method.
        check(vis.m_poly.Enum(&vis, pick));

        scan.SetMeshChange(i, LXf_MESHEDIT_GEOMETRY);
    }

    scan.Apply();
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupCount(unsigned int* count)
{
    count[0] = 3;
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupName(unsigned int index, const char** name)
{
    const static char* names[] = {"newPoly",
                            "newEdge",
                            "newVerx"};
    name[0] = names[index];
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupUserName(unsigned int index, const char** username)
{
    const static char* names[] = {"@tool.cdt@newPoly@0",
                            "@tool.cdt@newEdge@",
                            "@tool.cdt@newVerx@"};
    username[0] = names[index];
    return LXe_OK;
}

LxResult CToolOp::eltgrp_TestPolygon(unsigned int index, LXtPolygonID polygon)
{
#if 0
    if (m_polygon_set.find(polygon) != m_polygon_set.end())
        return LXe_TRUE;
#endif
    return LXe_FALSE;
}

LxResult CToolOp::eltgrp_TestEdge(unsigned int index, LXtEdgeID edge)
{
#if 0
	LXtPointID   p0, p1;

    m_cedge.Select(edge);
    m_cedge.Endpoints(&p0, &p1);
    if (m_point_set.find(p0) == m_point_set.end())
        return LXe_FALSE;
    if (m_point_set.find(p1) == m_point_set.end())
        return LXe_FALSE;
#endif
    return LXe_TRUE;
}

LxResult CToolOp::eltgrp_TestPoint(unsigned int index, LXtPointID point)
{
#if 0
    if (m_point_set.find(point) != m_point_set.end())
        return LXe_TRUE;
#endif
    return LXe_FALSE;
}


/*
 * Export tool server.
 */
void initialize()
{
    CLxGenericPolymorph* srv;

    srv = new CLxPolymorph<CTool>;
    srv->AddInterface(new CLxIfc_Tool<CTool>);
    srv->AddInterface(new CLxIfc_ToolModel<CTool>);
    srv->AddInterface(new CLxIfc_Attributes<CTool>);
    srv->AddInterface(new CLxIfc_AttributesUI<CTool>);
    srv->AddInterface(new CLxIfc_ChannelUI<CTool>);
    srv->AddInterface(new CLxIfc_StaticDesc<CTool>);
    thisModule.AddServer(SRVNAME_TOOL, srv);

    srv = new CLxPolymorph<CToolOp>;
    srv->AddInterface(new CLxIfc_ToolOperation<CToolOp>);
    srv->AddInterface(new CLxIfc_MeshElementGroup<CToolOp>);
    lx::AddSpawner(SRVNAME_TOOLOP, srv);
}
