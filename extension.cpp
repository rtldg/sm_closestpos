/**
 * vim: set ts=4 :
 * =============================================================================
 * SourceMod Sample Extension
 * Copyright (C) 2004-2008 AlliedModders LLC.  All rights reserved.
 * =============================================================================
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License, version 3.0, as published by the
 * Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, AlliedModders LLC gives you permission to link the
 * code of this program (as well as its derivative works) to "Half-Life 2," the
 * "Source Engine," the "SourcePawn JIT," and any Game MODs that run on software
 * by the Valve Corporation.  You must obey the GNU General Public License in
 * all respects for all other code used.  Additionally, AlliedModders LLC grants
 * this exception to all derivative works.  AlliedModders LLC defines further
 * exceptions, found in LICENSE.txt (as of this writing, version JULY-31-2007),
 * or <http://www.sourcemod.net/license.php>.
 *
 * Version: $Id$
 */

// because I use code from nanoflann (pointcloud_kdd_radius.cpp & utils.h)
/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2016 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#include "extension.h"
#include "ICellArray.h"

#include <vector>
#include "nanoflann.hpp"
using namespace nanoflann;

ClosestPos g_Extension;		/**< Global singleton for extension's main interface */
SMEXT_LINK(&g_Extension);

HandleType_t g_ClosestPosType = 0;
HandleType_t g_ArrayListType = 0;
IdentityToken_t *g_pCoreIdent;
extern const sp_nativeinfo_t ClosestPosNatives[];

template <typename T>
struct PointCloud
{
	struct Point
	{
		T  x,y,z;
	};

	std::vector<Point>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

typedef KDTreeSingleIndexAdaptor<
	L2_Simple_Adaptor<float, PointCloud<float> >,
	PointCloud<float>,
	3 /*dimensions*/
	> my_kd_tree_t;

class KDTreeContainer
{
public:
	PointCloud<float> cloud;
	my_kd_tree_t *index;
	int startidx;
};

class ClosestPosTypeHandler : public IHandleTypeDispatch
{
public:
	void OnHandleDestroy(HandleType_t type, void *object)
	{
		KDTreeContainer *container = (KDTreeContainer *)object;
		delete container;
	}
};

ClosestPosTypeHandler g_ClosestPosTypeHandler;

#ifdef _WIN32
// https://github.com/alliedmodders/sourcemod/blob/b14c18ee64fc822dd6b0f5baea87226d59707d5a/core/logic/HandleSys.h#L103
struct QHandleType_Caster
{
	void *dispatch;
	unsigned int freeID;
	unsigned int children;
	TypeAccess typeSec;
};
// https://github.com/alliedmodders/sourcemod/blob/b14c18ee64fc822dd6b0f5baea87226d59707d5a/core/logic/HandleSys.h#L230
struct HandleSystem_Caster
{
	void *vtable;
	void *m_Handles;
	QHandleType_Caster *m_Types;
};
#endif

bool ClosestPos::SDK_OnLoad(char* error, size_t maxlength, bool late)
{
	if (!g_pHandleSys->FindHandleType("CellArray", &g_ArrayListType))
	{
		snprintf(error, maxlength, "failed to find handle type 'CellArray' (ArrayList)");
		return false;
	}

#ifdef _WIN32
	HandleSystem_Caster *blah = (HandleSystem_Caster *)g_pHandleSys;
	// g_ArrayListType doesn't work here???
	// I really have no idea what's going on. This is terrible....
	unsigned index = 512;
	g_pCoreIdent = blah->m_Types[index].typeSec.ident;
#else
	Dl_info info;
	// memutils is from sourcemod.logic.so so we can grab the module from it.
	dladdr(memutils, &info);
	void *sourcemod_logic = dlopen(info.dli_fname, RTLD_NOLOAD);

	if (!sourcemod_logic)
	{
		snprintf(error, maxlength, "dlopen failed on '%s'", info.dli_fname);
		return false;
	}

	IdentityToken_t **token = (IdentityToken_t **)memutils->ResolveSymbol(sourcemod_logic, "g_pCoreIdent");

	if (!token)
	{
		snprintf(error, maxlength, "failed to resolve symbol g_pCoreIdent");
		return false;
	}

	g_pCoreIdent = *token;
#endif

	if (!g_pCoreIdent)
	{
		snprintf(error, maxlength, "g_pCoreIdent is NULL");
		return false;
	}

	g_ClosestPosType = g_pHandleSys->CreateType("ClosestPos",
		&g_ClosestPosTypeHandler,
		0,
		NULL,
		NULL,
		myself->GetIdentity(),
		NULL);

	sharesys->AddNatives(myself, ClosestPosNatives);
	sharesys->RegisterLibrary(myself, "closestpos");
	return true;
}

void ClosestPos::SDK_OnUnload()
{
	g_pHandleSys->RemoveType(g_ClosestPosType, myself->GetIdentity());
}

#define asdfMIN(a,b) (((a)<(b))?(a):(b))
#define asdfMAX(a,b) (((a)>(b))?(a):(b))

static cell_t sm_CreateClosestPos(IPluginContext *pContext, const cell_t *params)
{
	ICellArray *pArray;
	Handle_t arraylist = params[1];
	cell_t offset = params[2];

	if (offset < 0)
		return pContext->ThrowNativeError("Offset must be 0 or greater (given %d)", offset);

	if (arraylist == BAD_HANDLE)
		return pContext->ThrowNativeError("Bad handle passed as ArrayList %x", arraylist);

	HandleError err;
	HandleSecurity sec(g_pCoreIdent, g_pCoreIdent);

	if ((err = handlesys->ReadHandle(arraylist, g_ArrayListType, &sec, (void **)&pArray))
		!= HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid ArrayList Handle %x (error %d)", arraylist, err);
	}

	auto size = pArray->size();
	cell_t startidx = 0;
	cell_t count = size;

	if (params[0] > 2)
	{
		startidx = params[3];
		count = params[4];

		if (startidx < 0 || startidx > ((cell_t)size-1))
		{
			return pContext->ThrowNativeError("startidx (%d) must be >=0 and less than the ArrayList size (%d)", startidx, size);
		}

		if (count < 1)
		{
			return pContext->ThrowNativeError("count must be 1 or greater (given %d)", count);
		}

		count = asdfMIN(count, (cell_t)size-startidx);
	}

	KDTreeContainer *container = new KDTreeContainer();
	container->startidx = startidx;
	container->cloud.pts.resize(count);

	for (int i = 0; i < count; i++)
	{
		cell_t *blk = pArray->at(startidx+i);
		container->cloud.pts[i].x = sp_ctof(blk[offset+0]);
		container->cloud.pts[i].y = sp_ctof(blk[offset+1]);
		container->cloud.pts[i].z = sp_ctof(blk[offset+2]);
	}

	container->index = new my_kd_tree_t(3 /*dimensions*/, container->cloud, KDTreeSingleIndexAdaptorParams(100 /* max leaf */));
	container->index->buildIndex();

	return g_pHandleSys->CreateHandle(g_ClosestPosType, 
		container, 
		pContext->GetIdentity(), 
		myself->GetIdentity(), 
		NULL);
}

static cell_t sm_Find(IPluginContext *pContext, const cell_t *params)
{
	KDTreeContainer *container;
	HandleError err;
	HandleSecurity sec(pContext->GetIdentity(), myself->GetIdentity());

	if ((err = handlesys->ReadHandle(params[1], g_ClosestPosType, &sec, (void **)&container))
		!= HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}

	cell_t *addr;
	pContext->LocalToPhysAddr(params[2], &addr);

	float out_dist_sqr;
	size_t num_results = 1;
	size_t ret_index = 0;
	float query_pt[3] = {sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	container->index->knnSearch(&query_pt[0], num_results, &ret_index, &out_dist_sqr);

	return container->startidx + ret_index;
}

extern const sp_nativeinfo_t ClosestPosNatives[] =
{
	{"ClosestPos.ClosestPos",      sm_CreateClosestPos},
	{"ClosestPos.Find",            sm_Find},
	{NULL, NULL}
};
