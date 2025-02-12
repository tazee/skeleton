//
// The procedural mesh modifier version of Straight-Skeleton mesh edit named "tool.skeleton.item"
//

#pragma once

#include <lxsdk/lxu_attributes.hpp>
#include <lxsdk/lxu_select.hpp>
#include <lxsdk/lxu_attributes.hpp>

#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_log.hpp>
#include <lxsdk/lx_plugin.hpp>
#include <lxsdk/lx_seltypes.hpp>
#include <lxsdk/lx_tool.hpp>
#include <lxsdk/lx_toolui.hpp>
#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lx_vector.hpp>
#include <lxsdk/lx_pmodel.hpp>
#include <lxsdk/lx_vmodel.hpp>
#include <lxsdk/lx_channelui.hpp>
#include <lxsdk/lx_draw.hpp>
#include <lxsdk/lx_handles.hpp>

#include <lxsdk/lx_value.hpp>
#include <lxsdk/lx_select.hpp>
#include <lxsdk/lx_seltypes.hpp>

#include "skeleton.hpp"

using namespace lx_err;

#define SRVNAME_TOOL   "tool.skeleton"
#define SRVNAME_TOOLOP "toolop.skeleton"

#define ATTRs_MODE   "mode"
#define ATTRs_OFFSET "offset"
#define ATTRs_SHIFT  "shift"
#define ATTRs_STEPS  "steps"
#define ATTRs_HEIGHT "height"
#define ATTRs_SCALE  "scale"
#define ATTRs_MERGE  "merge"

#define ATTRa_MODE     0
#define ATTRa_OFFSET   1
#define ATTRa_SHIFT    2
#define ATTRa_STEPS    3
#define ATTRa_HEIGHT   4
#define ATTRa_SCALE    5
#define ATTRa_MERGE    6

#ifndef LXx_OVERRIDE
#define LXx_OVERRIDE override
#endif

//
// The Tool Operation is evaluated by the procedural modeling system.
//
class CToolOp : public CLxImpl_ToolOperation, public CLxImpl_MeshElementGroup
{
	public:
        // ToolOperation Interface
		LxResult    top_Evaluate(ILxUnknownID vts)  LXx_OVERRIDE;
    
        // MeshElementGroup Interface
        LxResult	eltgrp_GroupCount	(unsigned int *count)				LXx_OVERRIDE;
        LxResult	eltgrp_GroupName	(unsigned int index, const char **name)		LXx_OVERRIDE;
        LxResult	eltgrp_GroupUserName	(unsigned int index, const char **username)	LXx_OVERRIDE;
        LxResult	eltgrp_TestPolygon	(unsigned int index, LXtPolygonID polygon)	LXx_OVERRIDE;
        LxResult    eltgrp_TestEdge(unsigned int index, LXtEdgeID edge)	LXx_OVERRIDE;
        LxResult    eltgrp_TestPoint(unsigned int index, LXtPointID point)	LXx_OVERRIDE;

        void        SetElementGroup(CVisitor& vis);

        CLxUser_FalloffPacket falloff;
        CLxUser_Subject2Packet subject;

        unsigned offset_view;
        unsigned offset_screen;
        unsigned offset_falloff;
        unsigned offset_subject;
        unsigned offset_input;

        int    m_mode;
        double m_offset;
        double m_shift;
        int    m_steps;
        double m_height;
        double m_scale;
        int    m_merge;
    
        CLxUser_Edge m_cedge;
        std::unordered_set<LXtPointID> m_point_set;
        std::unordered_set<LXtPolygonID> m_polygon_set;
        std::unordered_set<LXtPolygonID> m_sidepoly_set;
};

/*
 * Straight-skeleton tool operator. Basic tool and tool model methods are defined here. The
 * attributes interface is inherited from the utility class.
 */

class CTool : public CLxImpl_Tool, public CLxImpl_ToolModel, public CLxDynamicAttributes, public CLxImpl_ChannelUI
{
public:
    CTool();

    void        tool_Reset() LXx_OVERRIDE;
    LXtObjectID tool_VectorType() LXx_OVERRIDE;
    const char* tool_Order() LXx_OVERRIDE;
    LXtID4      tool_Task() LXx_OVERRIDE;
	LxResult	tool_GetOp(void **ppvObj, unsigned flags) LXx_OVERRIDE;

    unsigned    tmod_Flags() LXx_OVERRIDE;
    LxResult    tmod_Enable(ILxUnknownID obj) LXx_OVERRIDE;
	void		tmod_Draw       (ILxUnknownID vts, ILxUnknownID stroke, int flags) LXx_OVERRIDE;
	void		tmod_Test       (ILxUnknownID vts, ILxUnknownID stroke, int flags) LXx_OVERRIDE;
    LxResult    tmod_Down(ILxUnknownID vts, ILxUnknownID adjust) LXx_OVERRIDE;
    void        tmod_Move(ILxUnknownID vts, ILxUnknownID adjust) LXx_OVERRIDE;
    void        tmod_Up(ILxUnknownID vts, ILxUnknownID adjust) LXx_OVERRIDE;

    using CLxDynamicAttributes::atrui_UIHints;  // to distinguish from the overloaded version in CLxImpl_AttributesUI

    void        atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints) LXx_OVERRIDE;
    LxResult	atrui_DisableMsg (unsigned int index, ILxUnknownID msg) LXx_OVERRIDE;

    LxResult    cui_Enabled           (const char *channelName, ILxUnknownID msg, ILxUnknownID item, ILxUnknownID read)	LXx_OVERRIDE;
    LxResult    cui_DependencyCount   (const char *channelName, unsigned *count) LXx_OVERRIDE;
    LxResult    cui_DependencyByIndex (const char *channelName, unsigned index, LXtItemType *depItemType, const char **depChannelName) LXx_OVERRIDE;

    bool TestPolygon();

    CLxUser_LogService   s_log;
    CLxUser_LayerService s_layer;
    CLxUser_VectorType   v_type;
    CLxUser_SelectionService s_sel;

    unsigned offset_view;
    unsigned offset_screen;
    unsigned offset_falloff;
    unsigned offset_subject;
    unsigned offset_input;
    unsigned offset_event;
	unsigned offset_center;
    unsigned mode_select;
	
	LXtItemType m_itemType;

    static LXtTagInfoDesc descInfo[];
    double m_offset0, m_height0, m_shift0;
    int     m_part;
};

