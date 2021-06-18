//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2021 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.


#ifndef PX_PHYSICS_NX_SCENE_QUERY_FILTERING
#define PX_PHYSICS_NX_SCENE_QUERY_FILTERING
/** \addtogroup scenequery
@{
*/

#include "PxPhysXConfig.h"
#include "PxFiltering.h"
#include "PxQueryReport.h"
#include "PxClient.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxShape;
class PxRigidActor;
struct PxQueryHit;


/**
\brief Filtering flags for scene queries.   场景查询的过滤标志。
@see PxQueryFilterData.flags
*/
struct PxQueryFlag
{
	enum Enum
	{
		eSTATIC				= (1<<0),	//!< Traverse static shapes     遍历静态形状

		eDYNAMIC			= (1<<1),	//!< Traverse dynamic shapes    遍历动态形状

        // 运行 pre-intersection-test 过滤器（参见 #PxQueryFilterCallback::preFilter()）
		ePREFILTER			= (1<<2),	//!< Run the pre-intersection-test filter (see #PxQueryFilterCallback::preFilter())

        // 运行 post-intersection-test 过滤器（参见 #PxQueryFilterCallback::postFilter()）
		ePOSTFILTER			= (1<<3),	//!< Run the post-intersection-test filter (see #PxQueryFilterCallback::postFilter())

        // 一旦找到任何命中就中止遍历并通过 callback.block 返回它。
        // 有助于查询性能。 eTOUCH 和 eBLOCK hitTypes 都被视为具有此标志的命中。
		eANY_HIT			= (1<<4),	//!< Abort traversal as soon as any hit is found and return it via callback.block.
										//!< Helps query performance. Both eTOUCH and eBLOCK hitTypes are considered hits with this flag.

        // 所有点击都报告为触摸。 使用 eTOUCH 覆盖从用户过滤器返回的 eBLOCK。
        // 这也是一个可以提高查询性能的优化提示。
		eNO_BLOCK			= (1<<5),	//!< All hits are reported as touching. Overrides eBLOCK returned from user filters with eTOUCH.
										//!< This is also an optimization hint that may improve query performance.

		eRESERVED			= (1<<15)	//!< Reserved for internal use  保留供内部使用
	};
};
PX_COMPILE_TIME_ASSERT(PxQueryFlag::eSTATIC==(1<<0));
PX_COMPILE_TIME_ASSERT(PxQueryFlag::eDYNAMIC==(1<<1));

/**
\brief Flags typedef for the set of bits defined in PxQueryFlag.
*/
typedef PxFlags<PxQueryFlag::Enum,PxU16> PxQueryFlags;
PX_FLAGS_OPERATORS(PxQueryFlag::Enum,PxU16)

/**
\brief Classification of scene query hits (intersections).  场景查询命中（交叉点）的分类。

 - eNONE: Returning this hit type means that the hit should not be reported.
          返回此命中类型意味着不应报告该命中
 - eBLOCK: For all raycast, sweep and overlap queries the nearest eBLOCK type hit will always be returned in PxHitCallback::block member.
           对于所有光线投射、扫描和重叠查询，最近的 eBLOCK 类型命中将始终在 PxHitCallback::block 成员中返回。
 - eTOUCH: Whenever a raycast, sweep or overlap query was called with non-zero PxHitCallback::nbTouches and PxHitCallback::touches
		   parameters, eTOUCH type hits that are closer or same distance (touchDistance <= blockDistance condition)
		   as the globally nearest eBLOCK type hit, will be reported.
           每当使用非零 PxHitCallback::nbTouches 和 PxHitCallback::touches 参数调用光线投射、扫描或重叠查询时，
           与全局最近的 eBLOCK 类型命中距离更近或相同（touchDistance <= blockDistance 条件）的 eTOUCH 类型命中，将 被举报。
 - For example, to record all hits from a raycast query, always return eTOUCH.
   例如，要记录来自 raycast 查询的所有命中，请始终返回 eTOUCH。

All hits in overlap() queries are treated as if the intersection distance were zero.
This means the hits are unsorted and all eTOUCH hits are recorded by the callback even if an eBLOCK overlap hit was encountered.
Even though all overlap() blocking hits have zero length, only one (arbitrary) eBLOCK overlap hit is recorded in PxHitCallback::block.
All overlap() eTOUCH type hits are reported (zero touchDistance <= zero blockDistance condition).
重叠（）查询中的所有命中都被视为交叉距离为零。
这意味着命中是未排序的，即使遇到 eBLOCK 重叠命中，回调也会记录所有 eTOUCH 命中。
即使所有的overlap() 阻塞命中的长度都为零，在PxHitCallback::block 中只记录了一个（任意的）eBLOCK 重叠命中。
报告所有重叠（） eTOUCH 类型的命中（零接触距离 <= 零块距离条件）

For raycast/sweep/overlap calls with zero touch buffer or PxHitCallback::nbTouches member,
only the closest hit of type eBLOCK is returned. All eTOUCH hits are discarded.
对于具有零触摸缓冲区或 PxHitCallback::nbTouches 成员的 raycast/sweep/overlap 调用，仅返回 eBLOCK 类型的最接近的命中。 
丢弃所有 eTOUCH 命中。

@see PxQueryFilterCallback.preFilter PxQueryFilterCallback.postFilter PxScene.raycast PxScene.sweep PxScene.overlap
*/
struct PxQueryHitType
{
	enum Enum
	{
		eNONE	= 0,	//!< the query should ignore this shape
        
        // 形状上的命中会触及查询的交集几何，但不会阻止它
		eTOUCH	= 1,	//!< a hit on the shape touches the intersection geometry of the query but does not block it

        // 形状上的命中会阻止查询（不会阻止重叠查询）
        eBLOCK	= 2		//!< a hit on the shape blocks the query (does not block overlap queries)
	};
};

/**
\brief Scene query filtering data.  场景查询过滤数据

Whenever the scene query intersects a shape, filtering is performed in the following order:
每当场景查询与形状相交时，按以下顺序进行过滤:

\li For non-batched queries only:<br>If the data field is non-zero, and the bitwise-AND value of data AND the shape's
queryFilterData is zero, the shape is skipped
仅适用于非批量查询：<br>如果数据字段非零，并且数据的按位与值 AND 形状的 queryFilterData 为零，则跳过形状

\li If filter callbacks are enabled in flags field (see #PxQueryFlags) they will get invoked accordingly.
如果在标志字段中启用过滤器回调（请参阅#PxQueryFlags），它们将被相应地调用。

\li If neither #PxQueryFlag::ePREFILTER or #PxQueryFlag::ePOSTFILTER is set, the hit defaults
to type #PxQueryHitType::eBLOCK when the value of PxHitCallback::nbTouches provided with the query is zero and to type
#PxQueryHitType::eTOUCH when PxHitCallback::nbTouches is positive.
如果 #PxQueryFlag::ePREFILTER 或 #PxQueryFlag::ePOSTFILTER 均未设置，则当查询提供的 PxHitCallback::nbTouches 的值为零时，
命中默认键入 #PxQueryHitType::eBLOCK 并在 PxHitCallback 时键入 #PxQueryHitType::eTOUCH ::nbTouches 是积极的。

@see PxScene.raycast PxScene.sweep PxScene.overlap PxBatchQuery.raycast PxBatchQuery.sweep PxBatchQuery.overlap PxQueryFlag::eANY_HIT
*/
struct PxQueryFilterData
{
	/** \brief default constructor */
	explicit PX_INLINE PxQueryFilterData() : flags(PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC)		{}

	/** \brief constructor to set both filter data and filter flags */
	explicit PX_INLINE PxQueryFilterData(const PxFilterData& fd, PxQueryFlags f) : data(fd), flags(f)	{}

	/** \brief constructor to set filter flags only */
	explicit PX_INLINE PxQueryFilterData(PxQueryFlags f) : flags(f)										{}

	PxFilterData	data;		//!< Filter data associated with the scene query
	PxQueryFlags	flags;		//!< Filter flags (see #PxQueryFlags)
};

/**
\brief Scene query filtering callbacks. 场景查询过滤回调

Custom filtering logic for scene query intersection candidates. If an intersection candidate object passes the data based filter
(see #PxQueryFilterData), filtering callbacks are executed if requested (see #PxQueryFilterData.flags)
场景查询交叉点候选者的自定义过滤逻辑。 如果交叉点候选对象通过基于数据的过滤器（请参阅#PxQueryFilterData），则在请求时执行过滤回调（请参阅#PxQueryFilterData.flags）

\li If #PxQueryFlag::ePREFILTER is set, the preFilter function runs before exact intersection tests.
If this function returns #PxQueryHitType::eTOUCH or #PxQueryHitType::eBLOCK, exact testing is performed to
determine the intersection location.
如果设置了 #PxQueryFlag::ePREFILTER，则 preFilter 函数在精确相交测试之前运行。
如果此函数返回#PxQueryHitType::eTOUCH 或#PxQueryHitType::eBLOCK，则执行精确测试以确定交叉点位置。

The preFilter function may overwrite the copy of queryFlags it receives as an argument to specify any of #PxHitFlag::eMODIFIABLE_FLAGS
on a per-shape basis. Changes apply only to the shape being filtered, and changes to other flags are ignored.
preFilter 函数可以覆盖它作为参数接收的 queryFlags 的副本，以在每个形状的基础上指定任何 #PxHitFlag::eMODIFIABLE_FLAGS。 
更改仅适用于被过滤的形状，对其他标志的更改将被忽略。

\li If #PxQueryFlag::ePREFILTER is not set, precise intersection testing is performed using the original query's filterData.flags.
如果未设置 #PxQueryFlag::ePREFILTER，则使用原始查询的 filterData.flags 执行精确的交集测试。

\li If #PxQueryFlag::ePOSTFILTER is set, the postFilter function is called for each intersection to determine the touch/block status.
This overrides any touch/block status previously returned from the preFilter function for this shape.
如果设置了 #PxQueryFlag::ePOSTFILTER，则为每个交叉点调用 postFilter 函数以确定触摸/阻止状态。
这将覆盖先前从此形状的 preFilter 函数返回的任何触摸/阻止状态。

Filtering calls are not guaranteed to be sorted along the ray or sweep direction.
过滤调用不能保证沿射线或扫描方向排序。

@see PxScene.raycast PxScene.sweep PxScene.overlap PxQueryFlags PxHitFlags
*/
class PxQueryFilterCallback
{
public:

	/**
	\brief This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.
           如果设置了 PxQueryFlag::ePREFILTER 标志，则在精确交集测试之前执行此过滤器回调。
	\param[in] filterData custom filter data specified as the query's filterData.data parameter.
	\param[in] shape A shape that has not yet passed the exact intersection test.
	\param[in] actor The shape's actor.
	\param[in,out] queryFlags scene query flags from the query's function call (only flags from PxHitFlag::eMODIFIABLE_FLAGS bitmask can be modified)
	\return the updated type for this hit  (see #PxQueryHitType)
	*/
	virtual PxQueryHitType::Enum preFilter(
		const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags) = 0;

	/**
	\brief This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.
           如果精确的交集测试返回 true 并且设置了 PxQueryFlag::ePOSTFILTER 标志，则执行此过滤器回调。
	\param[in] filterData custom filter data of the query
	\param[in] hit Scene query hit information. faceIndex member is not valid for overlap queries. For sweep and raycast queries the hit information can be cast to #PxSweepHit and #PxRaycastHit respectively.
	\return the updated hit type for this hit  (see #PxQueryHitType)
	*/
	virtual PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit) = 0;

	/**
	\brief virtual destructor
	*/
	virtual ~PxQueryFilterCallback() {}
};

/**
\brief Batched query pre-filter shader.     批量查询预过滤着色器

Custom filtering logic for batched query intersection candidates. If an intersection candidate object passes the data based filter (see #PxQueryFilterData),
filtering shader runs if specified in filtering flags (see #PxQueryFilterData.flags)
批量查询交叉候选的自定义过滤逻辑。 
如果交叉点候选对象通过基于数据的过滤器（请参阅#PxQueryFilterData），则过滤着色器会在过滤标志中指定时运行（请参阅#PxQueryFilterData.flags）

\li If #PxQueryFlag::ePREFILTER is set, the preFilter shader runs before exact intersection tests.
If the shader returns #PxQueryHitType::eTOUCH or #PxQueryHitType::eBLOCK, exact testing is performed to
determine the intersection location.
如果设置了 #PxQueryFlag::ePREFILTER，则 preFilter 着色器在精确相交测试之前运行。
如果着色器返回#PxQueryHitType::eTOUCH 或#PxQueryHitType::eBLOCK，则执行精确测试以确定相交位置。

The preFilter shader may overwrite the copy of queryFlags it receives as an argument to specify any of #PxHitFlag::eMODIFIABLE_FLAGS
on a per-shape basis. Changes apply only to the shape being filtered, and changes to other flags are ignored.
preFilter 着色器可以覆盖它作为参数接收的 queryFlags 的副本，以在每个形状的基础上指定任何 #PxHitFlag::eMODIFIABLE_FLAGS。 
更改仅适用于被过滤的形状，对其他标志的更改将被忽略。

\li If #PxQueryFlag::ePREFILTER is not set, precise intersection testing is performed using the original query's filterData.flags.
如果未设置 #PxQueryFlag::ePREFILTER，则使用原始查询的 filterData.flags 执行精确的交集测试。

Filtering calls are not guaranteed to be sorted along the ray or sweep direction.
过滤调用不能保证沿射线或扫描方向排序

\deprecated The batched query feature has been deprecated in PhysX version 3.4
PhysX 3.4 版中已弃用批处理查询功能

@see PxBatchQueryDesc.preFilterShader PxQueryFilterCallback.preFilter PxBatchQueryPostFilterShader

*/

/**
\param[in] queryFilterData Query filter data
\param[in] objectFilterData Object filter data
\param[in] constantBlock Global constant filter data (see #PxBatchQuery)
\param[in] constantBlockSize Size of global filter data (see #PxBatchQuery)
\param[in,out] hitFlags Per-object modifiable hit flags (only flags from PxHitFlag::eMODIFIABLE_FLAGS mask can be modified)
\return the updated hit type for this hit (see #PxQueryHitType)

@see PxBatchQueryPostFilterShader
*/
typedef PX_DEPRECATED PxQueryHitType::Enum (*PxBatchQueryPreFilterShader)(
	PxFilterData queryFilterData, PxFilterData objectFilterData,
	const void* constantBlock, PxU32 constantBlockSize,
	PxHitFlags& hitFlags);

/**
\brief Batched query post-filter shader.    批处理查询后过滤器着色器。

Custom filtering logic for batched query intersection candidates. If an intersection candidate object passes the data based filter (see #PxQueryFilterData),
the filtering shader run on request (see #PxQueryFilterData.flags)

\li If #PxQueryFlag::ePOSTFILTER is set, the postFilter shader is called for each intersection to determine the touch/block status.
This overrides any touch/block status previously returned from the preFilter function for this shape.

Filtering shaders are not in order along the query direction, rather they are processed in the order in which
candidate shapes for testing are found by PhysX' scene traversal algorithms.

\deprecated The batched query feature has been deprecated in PhysX version 3.4
PhysX 3.4 版本中已弃用批量查询功能

@see PxBatchQueryDesc.postFilterShader PxQueryFilterCallback.postFilter PxBatchQueryPreFilterShader
*/

/**
\param[in] queryFilterData Query filter data
\param[in] objectFilterData Object filter data
\param[in] constantBlock Global constant filter data (see #PxBatchQuery)
\param[in] constantBlockSize Size of global filter data (see #PxBatchQuery)
\param[in] hit Hit data from the prior exact intersection test.
\return the new hit type for this hit (see #PxQueryHitType)

@see PxBatchQueryPreFilterShader
*/

typedef PX_DEPRECATED PxQueryHitType::Enum (*PxBatchQueryPostFilterShader)(
	PxFilterData queryFilterData, PxFilterData objectFilterData,
	const void* constantBlock, PxU32 constantBlockSize,
	const PxQueryHit& hit);

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
