/*
 * TOOL.CPP  Polygon edit tool and tool operation using straight-skeleton algorithm. 
 */

#include "tool.hpp"

#define HANDLE_OFFSET		10001
#define HANDLE_HEIGHT		10002
#define HANDLE_SHIFT		10003

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
        { Offset, "offset" }, 
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

    dyna_Add(ATTRs_MERGE, LXsTYPE_BOOLEAN);

    tool_Reset();

    sPkt.NewVectorType(LXsCATEGORY_TOOL, v_type);
    sPkt.AddPacket(v_type, LXsP_TOOL_VIEW_EVENT, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_SCREEN_EVENT, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_FALLOFF, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_SUBJECT2, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_INPUT_EVENT, LXfVT_GET);
	sPkt.AddPacket (v_type, LXsP_TOOL_EVENTTRANS,  LXfVT_GET);
	sPkt.AddPacket (v_type, LXsP_TOOL_ACTCENTER,   LXfVT_GET);

    offset_view = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_VIEW_EVENT);
    offset_screen = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SCREEN_EVENT);
    offset_falloff = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_FALLOFF);
    offset_subject = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SUBJECT2);
    offset_input = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_INPUT_EVENT);
	offset_event  = sPkt.GetOffset (LXsCATEGORY_TOOL, LXsP_TOOL_EVENTTRANS);
	offset_center = sPkt.GetOffset (LXsCATEGORY_TOOL, LXsP_TOOL_ACTCENTER);
    mode_select = sMesh.SetMode("select");

    m_part    = -1;
    m_offset0 = 0.0;
}

/*
 * Reset sets the attributes back to defaults.
 */
void CTool::tool_Reset()
{
    CSkeleton ss;
    dyna_Value(ATTRa_MODE).SetInt(Offset);
    dyna_Value(ATTRa_OFFSET).SetFlt(ss.m_offset);
    dyna_Value(ATTRa_SHIFT).SetFlt(ss.m_shift);
    dyna_Value(ATTRa_STEPS).SetInt(ss.m_steps);
    dyna_Value(ATTRa_HEIGHT).SetFlt(ss.m_height);
    dyna_Value(ATTRa_SCALE).SetFlt(ss.m_scale);
    dyna_Value(ATTRa_MERGE).SetFlt(ss.m_merge);
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
    dyna_Value(ATTRa_MERGE).GetInt(&toolop->m_merge);

    toolop->offset_view = offset_view;
    toolop->offset_screen = offset_screen;
    toolop->offset_falloff = offset_falloff;
    toolop->offset_subject = offset_subject;
    toolop->offset_input = offset_input;

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
    return LXfTMOD_I0_INPUT | LXfTMOD_DRAW_3D;
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

void CTool::tmod_Draw (ILxUnknownID vts, ILxUnknownID stroke, int flags)
{
	CLxUser_VectorStack	vec (vts);
	CLxUser_StrokeDraw	draw (stroke);
	CLxUser_HandleDraw	handle (stroke);
	LXtVector		    pos;
    LXtMatrix           mat;
	int			        dFlags, mode;

	LXpToolActionCenter	*acen = (LXpToolActionCenter *) vec.Read (offset_center);
	LXpToolViewEvent	*view = (LXpToolViewEvent *) vec.Read (offset_view);
	if (!view || ((view->type != LXi_VIEWTYPE_3D)))
		return;

    dyna_Value(ATTRa_MODE).GetInt(&mode);

	//
	// Draw the offset handle.
	//
	dFlags = LXi_THANDf_SMALL;

    lx::MatrixIdent(mat);
    switch (mode)
    {
        case Offset:
            if (m_part == HANDLE_OFFSET)
                dFlags |= LXi_THANDf_HOT;
            handle.ScaleHandle(acen->v, mat, 0, HANDLE_OFFSET, m_offset0, 1, dFlags);
            break;

        case Extrude:
            if (m_part == HANDLE_HEIGHT)
                dFlags |= LXi_THANDf_HOT;
            handle.ScaleHandle(acen->v, mat, 1, HANDLE_HEIGHT, m_height0, 1, dFlags);
            break;

        case Duplicate:
            if (m_part == HANDLE_OFFSET)
                dFlags |= LXi_THANDf_HOT;
            handle.ScaleHandle(acen->v, mat, 0, HANDLE_OFFSET, m_offset0, 1, dFlags);
            dFlags = LXi_THANDf_SMALL;
            if (m_part == HANDLE_SHIFT)
                dFlags |= LXi_THANDf_HOT;
            handle.ScaleHandle(acen->v, mat, 1, HANDLE_SHIFT, m_shift0, 1, dFlags);
            break;
    }
}

void CTool::tmod_Test (ILxUnknownID vts, ILxUnknownID stroke, int flags)
{
    tmod_Draw(vts, stroke, flags);
}

LxResult CTool::tmod_Down(ILxUnknownID vts, ILxUnknownID adjust)
{
	CLxUser_AdjustTool	 at (adjust);
	CLxUser_VectorStack	 vec (vts);
	LXpToolActionCenter* acen = (LXpToolActionCenter *) vec.Read (offset_center);
	LXpToolInputEvent*   ipkt = (LXpToolInputEvent *) vec.Read (offset_input);

	CLxUser_EventTranslatePacket epkt;
	vec.ReadObject (offset_event, epkt);

	m_part = ipkt->part;

	switch (ipkt->part) {
	    case HANDLE_OFFSET:
            dyna_Value(ATTRa_OFFSET).GetFlt(&m_offset0);
		    epkt.HitHandle (vts, acen->v);
		    break;

	    case HANDLE_HEIGHT:
            dyna_Value(ATTRa_HEIGHT).GetFlt(&m_height0);
		    epkt.HitHandle (vts, acen->v);
		    break;

        case HANDLE_SHIFT:
            dyna_Value(ATTRa_SHIFT).GetFlt(&m_shift0);
            epkt.HitHandle (vts, acen->v);
            break;
	}
    return LXe_TRUE;
}

void CTool::tmod_Move(ILxUnknownID vts, ILxUnknownID adjust)
{
    CLxUser_AdjustTool at(adjust);
    CLxUser_VectorStack vec(vts);
    LXpToolScreenEvent*  spak = static_cast<LXpToolScreenEvent*>(vec.Read(offset_screen));
	LXpToolActionCenter* acen = (LXpToolActionCenter *) vec.Read (offset_center);
	LXpToolInputEvent*   ipkt = (LXpToolInputEvent *) vec.Read (offset_input);

	CLxUser_EventTranslatePacket epkt;
	vec.ReadObject (offset_event, epkt);

	//
	// Get the new handle position.
	//
	LXtVector		 pos, delta;
	epkt.GetNewPosition (vts, pos);
	LXx_VSUB3 (delta, pos, acen->v);
    double len = LXx_VLEN(delta);

	switch (ipkt->part) {
	    case HANDLE_OFFSET:
            if (delta[0] < 0.0)
                len *= -1.0;
            at.SetFlt(ATTRa_OFFSET, len + m_offset0);
		    break;

	    case HANDLE_HEIGHT:
            if (delta[1] < 0.0)
                len *= -1.0;
            at.SetFlt(ATTRa_HEIGHT, len + m_height0);
		    break;

        case HANDLE_SHIFT:
            if (delta[1] < 0.0)
                len *= -1.0;
            at.SetFlt(ATTRa_SHIFT, len + m_shift0);
            break;
	}
}

void CTool::tmod_Up(ILxUnknownID vts, ILxUnknownID adjust)
{
    m_part = -1;
    m_offset0 = 0.0;
    m_height0 = 0.0;
    m_shift0 = 0.0;
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

LxResult CTool::atrui_DisableMsg (unsigned int index, ILxUnknownID msg)
{
    CLxUser_Message		 message (msg);

    int mode;
    dyna_Value(ATTRa_MODE).GetInt(&mode);

    switch (index) {
        case ATTRa_OFFSET:
            if (mode != Offset && mode != Duplicate)
            {
                message.SetCode (LXe_DISABLED);
                message.SetMessage ("tool.skeleton", "OnlyOffsetOrDuplicate", 0);
                return LXe_DISABLED;
            }
            break;
        case ATTRa_SHIFT:
        case ATTRa_STEPS:
            if (mode != Duplicate)
            {
                message.SetCode (LXe_DISABLED);
                message.SetMessage ("tool.skeleton", "OnlyDuplicate", 0);
                return LXe_DISABLED;
            }
            break;
        case ATTRa_SCALE:
        case ATTRa_HEIGHT:
            if (mode != Extrude)
            {
                message.SetCode (LXe_DISABLED);
                message.SetMessage ("tool.skeleton", "OnlyExtrude", 0);
                return LXe_DISABLED;
            }
            break;
        case ATTRa_MERGE:
            if (mode != Offset)
            {
                message.SetCode (LXe_DISABLED);
                message.SetMessage ("tool.skeleton", "OnlyOffset", 0);
                return LXe_DISABLED;
            }
            break;
    }
    return LXe_OK;
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

LxResult CTool::cui_Enabled (const char *channelName, ILxUnknownID msg_obj, ILxUnknownID item_obj, ILxUnknownID read_obj)
{
	CLxUser_Item	 	 item (item_obj);
	CLxUser_ChannelRead	 chan_read (read_obj);

    std::string name(channelName);

	if (name == ATTRs_OFFSET)
    {
        if ((chan_read.IValue (item, ATTRs_MODE) != Offset)
         && (chan_read.IValue (item, ATTRs_MODE) != Duplicate))
		    return LXe_CMD_DISABLED;
    }
	else if (name == ATTRs_SHIFT)
    {
        if (chan_read.IValue (item, ATTRs_MODE) != Duplicate)
		    return LXe_CMD_DISABLED;
    }
	else if (name == ATTRs_STEPS)
    {
        if (chan_read.IValue (item, ATTRs_MODE) != Duplicate)
		    return LXe_CMD_DISABLED;
    }
	else if (name == ATTRs_HEIGHT)
    {
        if (chan_read.IValue (item, ATTRs_MODE) != Extrude)
		    return LXe_CMD_DISABLED;
    }
	else if (name == ATTRs_SCALE)
    {
        if (chan_read.IValue (item, ATTRs_MODE) != Extrude)
		    return LXe_CMD_DISABLED;
    }
	else if (name == ATTRs_MERGE)
    {
        if (chan_read.IValue (item, ATTRs_MODE) != Offset)
		    return LXe_CMD_DISABLED;
    }
	
	return LXe_OK;
}

LxResult CTool::cui_DependencyCount (const char *channelName, unsigned *count)
{
	count[0] = 0;

	if (std::string(channelName) == ATTRs_OFFSET)
		count[0] = 1;
	else if (std::string(channelName) == ATTRs_SHIFT)
		count[0] = 1;
	else if (std::string(channelName) == ATTRs_STEPS)
		count[0] = 1;
	else if (std::string(channelName) == ATTRs_HEIGHT)
		count[0] = 1;
	else if (std::string(channelName) == ATTRs_SCALE)
		count[0] = 1;
    else if (std::string(channelName) == ATTRs_MERGE)
        count[0] = 1;
	
	return LXe_OK;
}

LxResult CTool::cui_DependencyByIndex (const char *channelName, unsigned index, LXtItemType *depItemType, const char **depChannel)
{
	depItemType[0] = m_itemType;
	
	if (std::string(channelName) == ATTRs_OFFSET)
	{
		depChannel[0] = ATTRs_MODE;
		return LXe_OK;
	}	
	else if (std::string(channelName) == ATTRs_SHIFT)
	{
		depChannel[0] = ATTRs_MODE;
		return LXe_OK;
	}	
	else if (std::string(channelName) == ATTRs_STEPS)
	{
		depChannel[0] = ATTRs_MODE;
		return LXe_OK;
	}	
	else if (std::string(channelName) == ATTRs_HEIGHT)
	{
		depChannel[0] = ATTRs_MODE;
		return LXe_OK;
	}	
	else if (std::string(channelName) == ATTRs_SCALE)
	{
		depChannel[0] = ATTRs_MODE;
		return LXe_OK;
	}	
	else if (std::string(channelName) == ATTRs_MERGE)
	{
		depChannel[0] = ATTRs_MODE;
		return LXe_OK;
	}
		
	return LXe_OUTOFBOUNDS;
}

/*
 * Tool evaluation uses layer scan interface to walk through all the active
 * meshes and visit all the selected polygons.
 */
LxResult CToolOp::top_Evaluate(ILxUnknownID vts)
{
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
        vis.m_skeleton.m_merge  = m_merge;

        check(vis.m_vert.fromMesh(vis.edit_mesh));
        check(vis.m_poly.fromMesh(vis.edit_mesh));

        // Triangulate selected polygons using CDT traingulation method.
        check(vis.m_poly.Enum(&vis, pick));

        // Cache extruded polygons for element group
        SetElementGroup(vis);

        scan.SetMeshChange(i, LXf_MESHEDIT_GEOMETRY);
    }

    scan.Apply();
    return LXe_OK;
}

void CToolOp::SetElementGroup(CVisitor& vis)
{
    m_polygon_set.insert(vis.top_polygons.begin(), vis.top_polygons.end());
    for (auto pol : vis.top_polygons)
    {
        unsigned count;
        vis.m_poly.Select(pol);
        vis.m_poly.VertexCount(&count);
        for (auto i = 0u; i < count; i++)
        {
            LXtPointID pnt;
            vis.m_poly.VertexByIndex(i, &pnt);
            m_point_set.insert(pnt);
        }
    }
    m_sidepoly_set.insert(vis.side_polygons.begin(), vis.side_polygons.end());
    for (auto pol : vis.side_polygons)
    {
        unsigned count;
        vis.m_poly.Select(pol);
        vis.m_poly.VertexCount(&count);
        for (auto i = 0u; i < count; i++)
        {
            LXtPointID pnt;
            vis.m_poly.VertexByIndex(i, &pnt);
            m_point_set.insert(pnt);
        }
    }
    vis.edit_mesh.GetEdges(m_cedge);
}

LxResult CToolOp::eltgrp_GroupCount(unsigned int* count)
{
    count[0] = 4;
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupName(unsigned int index, const char** name)
{
    const static char* names[] = {"newPoly",
                            "newEdge",
                            "newVerx",
                            "sidePoly"};
    name[0] = names[index];
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupUserName(unsigned int index, const char** username)
{
    const static char* names[] = {"@tool.skeleton@newPoly@0",
                            "@tool.skeleton@newEdge@",
                            "@tool.skeleton@newVerx@",
                            "@tool.skeleton@sidePoly@"};
    username[0] = names[index];
    return LXe_OK;
}

LxResult CToolOp::eltgrp_TestPolygon(unsigned int index, LXtPolygonID polygon)
{
    if (index == 0)
    {
        if (m_polygon_set.find(polygon) != m_polygon_set.end())
            return LXe_TRUE;
    }
    else if (index == 3)
    {
        if (m_sidepoly_set.find(polygon) != m_sidepoly_set.end())
            return LXe_TRUE;
    }
    return LXe_FALSE;
}

LxResult CToolOp::eltgrp_TestEdge(unsigned int index, LXtEdgeID edge)
{
	LXtPointID   p0, p1;

    m_cedge.Select(edge);
    m_cedge.Endpoints(&p0, &p1);
    if (m_point_set.find(p0) == m_point_set.end())
        return LXe_FALSE;
    if (m_point_set.find(p1) == m_point_set.end())
        return LXe_FALSE;
    return LXe_TRUE;
}

LxResult CToolOp::eltgrp_TestPoint(unsigned int index, LXtPointID point)
{
    if (m_point_set.find(point) != m_point_set.end())
        return LXe_TRUE;
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
