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


#ifndef PX_PHYSICS_NX_SCENE
#define PX_PHYSICS_NX_SCENE
/** \addtogroup physics
@{
*/

#include "PxVisualizationParameter.h"
#include "PxSceneDesc.h"
#include "PxSimulationStatistics.h"
#include "PxQueryReport.h"
#include "PxQueryFiltering.h"
#include "PxClient.h"
#include "task/PxTask.h"

#include "pvd/PxPvdSceneClient.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxRigidStatic;
class PxRigidDynamic;
class PxConstraint;
class PxMaterial;
class PxSimulationEventCallback;
class PxPhysics;
class PxBatchQueryDesc;
class PxBatchQuery;
class PxAggregate;
class PxRenderBuffer;

class PxSphereGeometry;
class PxBoxGeometry;
class PxCapsuleGeometry;

class PxPruningStructure;
class PxBVHStructure;
struct PxContactPairHeader;

typedef PxU8 PxDominanceGroup;

class PxPvdSceneClient;

/**
\brief Expresses the dominance relationship of a contact.   表示两者之间联系的支配关系。
For the time being only three settings are permitted:       目前只允许三种设置：
(1, 1), (0, 1), and (1, 0).                                 
@see getDominanceGroup() PxDominanceGroup PxScene::setDominanceGroupPair()
*/	
struct PxDominanceGroupPair
{
	PxDominanceGroupPair(PxU8 a, PxU8 b) 
		: dominance0(a), dominance1(b) {}
	PxU8 dominance0;
	PxU8 dominance1;
};


/**
\brief Identifies each type of actor for retrieving actors from a scene.
       标识用于从场景中检索演员的每种类型的演员。
\note #PxArticulationLink objects are not supported. Use the #PxArticulation object to retrieve all its links.
      不支持 #PxArticulationLink 对象。 使用 #PxArticulation 对象检索其所有链接。
@see PxScene::getActors(), PxScene::getNbActors()
*/
struct PxActorTypeFlag
{
	enum Enum
	{
		/**
		\brief A static rigid body
		@see PxRigidStatic
		*/
		eRIGID_STATIC		= (1 << 0),

		/**
		\brief A dynamic rigid body
		@see PxRigidDynamic
		*/
		eRIGID_DYNAMIC		= (1 << 1)
	};
};

/**
\brief Collection of set bits defined in PxActorTypeFlag.   // PxActorTypeFlag 中定义的设置位的集合。
@see PxActorTypeFlag
*/
typedef PxFlags<PxActorTypeFlag::Enum,PxU16> PxActorTypeFlags;
PX_FLAGS_OPERATORS(PxActorTypeFlag::Enum,PxU16)

/**
\brief single hit cache for scene queries.  // 用于场景查询的单次命中缓存。

If a cache object is supplied to a scene query, the cached actor/shape pair is checked for intersection first.
如果缓存对象提供给场景查询，则首先检查缓存的actor/shape 对是否有交集。
\note Filters are not executed for the cached shape.
      不会为缓存的形状执行过滤器。
\note If intersection is found, the hit is treated as blocking.
      如果找到交叉点，则将命中视为阻塞。
\note Typically actor and shape from the last PxHitCallback.block query result is used as a cached actor/shape pair.
      通常来自最后一个 PxHitCallback.block 查询结果的 actor 和 shape 被用作缓存的 actor/shape 对。
\note Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
      使用过去的触摸命中作为缓存会产生不正确的行为，因为缓存的命中将始终被视为阻塞。
\note Cache is only used if no touch buffer was provided, for single nearest blocking hit queries and queries using eANY_HIT flag.
      缓存仅在未提供触摸缓冲区的情况下使用，用于单个最近的阻塞命中查询和使用 eANY_HIT 标志的查询。
\note if non-zero touch buffer was provided, cache will be ignored
      如果提供了非零触摸缓冲区，缓存将被忽略
\note It is the user's responsibility to ensure that the shape and actor are valid, so care must be taken
when deleting shapes to invalidate cached references.
      用户有责任确保形状和角色有效，因此在删除形状以使缓存的引用无效时必须小心。
The faceIndex field is an additional hint for a mesh or height field which is not currently used.
faceIndex 字段是当前未使用的网格或高度字段的附加提示。
@see PxScene.raycast
*/
struct PxQueryCache
{
	/**
	\brief constructor sets to default 
	*/
	PX_INLINE PxQueryCache() : shape(NULL), actor(NULL), faceIndex(0xffffffff) {}

	/**
	\brief constructor to set properties
	*/
	PX_INLINE PxQueryCache(PxShape* s, PxU32 findex) : shape(s), actor(NULL), faceIndex(findex) {}

	PxShape*		shape;			//!< Shape to test for intersection first   // 首先测试相交的形状
	PxRigidActor*	actor;			//!< Actor to which the shape belongs       // 形状所属的actor
	PxU32			faceIndex;		//!< Triangle index to test first - NOT CURRENTLY SUPPORTED // 首先要测试的三角形索引 - 目前不支持
};

/** 
 \brief A scene is a collection of bodies and constraints which can interact.
        场景是可以交互的物体和约束的集合

 The scene simulates the behavior of these objects over time. Several scenes may exist 
 at the same time, but each body or constraint is specific to a scene 
 -- they may not be shared.
 场景模拟这些对象随时间的行为。 多个场景可能同时存在，但每个主体或约束都特定于一个场景——它们可能无法共享。

 @see PxSceneDesc PxPhysics.createScene() release()
*/
class PxScene
{
	protected:
	
	/************************************************************************************************/

	/** @name Basics        // 基础接口
	*/
	//@{
	
								PxScene(): userData(0)	{}
	virtual						~PxScene()	{}

	public:

	/**
	\brief Deletes the scene.

	Removes any actors and constraint shaders from this scene(if the user hasn't already done so).
    从此场景中移除所有角色和约束着色器（如果用户尚未这样做）。

	Be sure	to not keep a reference to this object after calling release.
	Avoid release calls while the scene is simulating (in between simulate() and fetchResults() calls).
    确保在调用 release 后不要保留对这个对象的引用。
    避免在场景模拟时释放调用（在调用simulate() 和fetchResults() 之间）。
	
	@see PxPhysics.createScene() 
	*/
	virtual		void			release() = 0;

	/**
	\brief Sets a scene flag. You can only set one flag at a time.  设置场景标志。 您一次只能设置一个标志。
	\note Not all flags are mutable and changing some will result in an error. Please check #PxSceneFlag to see which flags can be changed.
          并非所有标志都是可变的，更改某些标志会导致错误。 请检查 #PxSceneFlag 以查看可以更改哪些标志。
	@see PxSceneFlag
	*/
	virtual		void			setFlag(PxSceneFlag::Enum flag, bool value) = 0;

	/**
	\brief Get the scene flags.
	\return The scene flags. See #PxSceneFlag
	@see PxSceneFlag
	*/
	virtual		PxSceneFlags	getFlags() const = 0;


	/**
	\brief Set new scene limits. 
	\note Increase the maximum capacity of various data structures in the scene. The new capacities will be 
	at least as large as required to deal with the objects currently in the scene. Further, these values 
	are for preallocation and do not represent hard limits.
    增加场景中各种数据结构的最大容量。 新容量将至少与处理当前场景中的对象所需的一样大。 此外，这些值用于预分配，并不代表硬限制。
	\param[in] limits Scene limits.
	@see PxSceneLimits
	*/
	virtual void				setLimits(const PxSceneLimits& limits) = 0;

	/**
	\brief Get current scene limits.
	\return Current scene limits.
	@see PxSceneLimits
	*/
	virtual PxSceneLimits		getLimits() const = 0;


	/**
	\brief Call this method to retrieve the Physics SDK.
	\return The physics SDK this scene is associated with.
	@see PxPhysics
	*/
	virtual	PxPhysics&			getPhysics() = 0;

	/**
	\brief Retrieves the scene's internal timestamp, increased each time a simulation step is completed.
           获取场景的内部时间戳，每次模拟步骤完成时都会增加
	\return scene timestamp
	*/
	virtual	PxU32				getTimestamp()	const	= 0;

	
	//@}
	/************************************************************************************************/

	/** @name Add/Remove Contained Objects  // 添加/删除包含的对象
	*/
	//@{
	/**
	\brief Adds an articulation to this scene.  // 添加关节
	\note If the articulation is already assigned to a scene (see #PxArticulation::getScene), the call is ignored and an error is issued.
          如果关节已分配给场景（请参阅#PxArticulation::getScene），则调用将被忽略并发出错误。
	\param[in] articulation Articulation to add to scene. See #PxArticulation
	@see PxArticulation
	*/
	virtual	void				addArticulation(PxArticulationBase& articulation) = 0;

	/**
	\brief Removes an articulation from this scene.
	\note If the articulation is not part of this scene (see #PxArticulation::getScene), the call is ignored and an error is issued. 
          如果关节不是此场景的一部分（请参阅#PxArticulation::getScene），则调用将被忽略并发出错误。
	\note If the articulation is in an aggregate it will be removed from the aggregate.
          如果关节在聚合中，它将从聚合中删除。
	\param[in] articulation Articulation to remove from scene. See #PxArticulation
	\param[in] wakeOnLostTouch Specifies whether touching objects from the previous frame should get woken up in the next frame. Only applies to PxArticulation and PxRigidActor types.
                    指定是否应在下一帧唤醒前一帧中的触摸对象。 仅适用于 PxArticulation 和 PxRigidActor 类型。
	@see PxArticulation, PxAggregate
	*/
	virtual	void				removeArticulation(PxArticulationBase& articulation, bool wakeOnLostTouch = true) = 0;


	//@}
	/************************************************************************************************/


	/**
	\brief Adds an actor to this scene.
	\note If the actor is already assigned to a scene (see #PxActor::getScene), the call is ignored and an error is issued.
          如果actor 已经分配给一个场景（参见#PxActor::getScene），调用将被忽略并发出错误。
	\note If the actor has an invalid constraint, in checked builds the call is ignored and an error is issued.
          如果actor 有一个无效的约束，在检查构建中调用将被忽略并发出错误。
	\note You can not add individual articulation links (see #PxArticulationLink) to the scene. Use #addArticulation() instead.
          您无法向场景添加单独的关节链接（请参阅 #PxArticulationLink）。 请改用 #addArticulation()。
	\note If the actor is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
	      it connects to another actor that is part of the scene already. 
          如果actor 是一个PxRigidActor，那么如果每个分配的PxConstraint 对象连接到另一个已经是场景一部分的actor，它就会自动添加到场景中。
	\note When BVHStructure is provided the actor shapes are grouped together. 
          The scene query pruning structure inside PhysX SDK will store/update one
          bound per actor. The scene queries against such an actor will query actor
          bounds and then make a local space query against the provided BVH structure, which is in
          actor's local space.
          当提供 BVHStructure 时，actor 形状被组合在一起。
          PhysX SDK 中的场景查询修剪结构将存储/更新每个actor 的一个绑定。 
          针对这样的actor 的场景查询将查询actor 边界，然后针对提供的BVH 结构进行本地空间查询，该结构位于actor 的本地空间中。
	\param[in] actor Actor to add to scene.
	\param[in] bvhStructure BVHStructure for actor shapes.
	@see PxActor, PxConstraint::isValid(), PxBVHStructure
	*/
	virtual	void				addActor(PxActor& actor, const PxBVHStructure* bvhStructure = NULL) = 0;

	/**
	\brief Adds actors to this scene.	
	\note If one of the actors is already assigned to a scene (see #PxActor::getScene), the call is ignored and an error is issued.
          如果其中一个演员已分配给场景（请参阅#PxActor::getScene），则该调用将被忽略并发出错误消息。
	\note You can not add individual articulation links (see #PxArticulationLink) to the scene. Use #addArticulation() instead.
	\note If an actor in the array contains an invalid constraint, in checked builds the call is ignored and an error is issued.
          如果数组中的参与者包含无效约束，则在检查构建中忽略调用并发出错误。
	\note If an actor in the array is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
	      it connects to another actor that is part of the scene already.
          如果数组中的 actor 是 PxRigidActor，那么如果每个分配的 PxConstraint 对象连接到另一个已经是场景一部分的 actor，它就会自动添加到场景中。
	\note this method is optimized for high performance, and does not support buffering. It may not be called during simulation.
          此方法针对高性能进行了优化，并且不支持缓冲。 它可能不会在模拟过程中被调用。
	\param[in] actors Array of actors to add to scene.
	\param[in] nbActors Number of actors in the array.
	@see PxActor, PxConstraint::isValid()
	*/
	virtual	void				addActors(PxActor*const* actors, PxU32 nbActors) = 0;

	/**
	\brief Adds a pruning structure together with its actors to this scene.	
           将修剪结构及其actor添加到此场景中。

	\note If an actor in the pruning structure contains an invalid constraint, in checked builds the call is ignored and an error is issued.
          如果修剪结构中的参与者包含无效约束，则在检查构建中忽略调用并发出错误。
	\note For all actors in the pruning structure each assigned PxConstraint object will get added to the scene automatically if
	it connects to another actor that is part of the scene already.
          对于修剪结构中的所有演员，如果每个分配的 PxConstraint 对象连接到另一个已经是场景一部分的演员，则它会自动添加到场景中。
	\note This method is optimized for high performance, and does not support buffering. It may not be called during simulation.
          此方法针对高性能进行了优化，不支持缓冲。 它可能不会在模拟过程中被调用。
	\note Merging a PxPruningStructure into an active scene query optimization AABB tree might unbalance the tree. A typical use case for 
	PxPruningStructure is a large world scenario where blocks of closely positioned actors get streamed in. The merge process finds the 
	best node in the active scene query optimization AABB tree and inserts the PxPruningStructure. Therefore using PxPruningStructure 
	for actors scattered throughout the world will result in an unbalanced tree.
          将 PxPruningStructure 合并到活动场景查询优化 AABB 树中可能会使树不平衡。 
          PxPruningStructure 的一个典型用例是一个大世界场景，其中紧密定位的 actor 块被流式传输。
          合并过程在活动场景查询优化 AABB 树中找到最佳节点并插入 PxPruningStructure。 
          因此，对分散在世界各地的演员使用 PxPruningStructure 将导致树不平衡。
	\param[in] pruningStructure Pruning structure for a set of actors.
	@see PxPhysics::createPruningStructure, PxPruningStructure
	*/
	virtual	void				addActors(const PxPruningStructure& pruningStructure) = 0;

	/**
	\brief Removes an actor from this scene.
	\note If the actor is not part of this scene (see #PxActor::getScene), the call is ignored and an error is issued.
	\note You can not remove individual articulation links (see #PxArticulationLink) from the scene. Use #removeArticulation() instead.
          您无法从场景中删除单个关节链接（请参阅 #PxArticulationLink）。 请改用 #removeArticulation()。
	\note If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
          如果 actor 是 PxRigidActor，则所有分配的 PxConstraint 对象将自动从场景中移除。
	\note If the actor is in an aggregate it will be removed from the aggregate.
          如果参与者在聚合中，它将从聚合中移除。
	\param[in] actor Actor to remove from scene.
	\param[in] wakeOnLostTouch Specifies whether touching objects from the previous frame should get woken up in the next frame. Only applies to PxArticulation and PxRigidActor types.
	@see PxActor, PxAggregate
	*/
	virtual	void				removeActor(PxActor& actor, bool wakeOnLostTouch = true) = 0;

	/**
	\brief Removes actors from this scene.
	\note If some actor is not part of this scene (see #PxActor::getScene), the actor remove is ignored and an error is issued.
	\note You can not remove individual articulation links (see #PxArticulationLink) from the scene. Use #removeArticulation() instead.
	\note If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
	\param[in] actors Array of actors to add to scene.
	\param[in] nbActors Number of actors in the array.
	\param[in] wakeOnLostTouch Specifies whether touching objects from the previous frame should get woken up in the next frame. Only applies to PxArticulation and PxRigidActor types.
	@see PxActor
	*/
	virtual	void				removeActors(PxActor*const* actors, PxU32 nbActors, bool wakeOnLostTouch = true) = 0;

	/**
	\brief Adds an aggregate to this scene.         // 向该场景添加聚合
	\note If the aggregate is already assigned to a scene (see #PxAggregate::getScene), the call is ignored and an error is issued.
          如果聚合已分配给场景（请参阅#PxAggregate::getScene），则忽略调用并发出错误。
	\note If the aggregate contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
          如果聚合包含具有无效约束的参与者，则在检查构建中忽略调用并发出错误。
	\note If the aggregate already contains actors, those actors are added to the scene as well.
          如果聚合已包含演员，则这些演员也会添加到场景中。
	\param[in] aggregate Aggregate to add to scene.
	@see PxAggregate, PxConstraint::isValid()
	*/
    virtual	void				addAggregate(PxAggregate& aggregate)	= 0;

	/**
	\brief Removes an aggregate from this scene.
	\note If the aggregate is not part of this scene (see #PxAggregate::getScene), the call is ignored and an error is issued.
	\note If the aggregate contains actors, those actors are removed from the scene as well.
	\param[in] aggregate Aggregate to remove from scene.
	\param[in] wakeOnLostTouch Specifies whether touching objects from the previous frame should get woken up in the next frame. Only applies to PxArticulation and PxRigidActor types.
                    指定是否应在下一帧唤醒前一帧中的触摸对象。 仅适用于 PxArticulation 和 PxRigidActor 类型。
	@see PxAggregate
	*/
	virtual	void				removeAggregate(PxAggregate& aggregate, bool wakeOnLostTouch = true)	= 0;

	/**
	\brief Adds objects in the collection to this scene.    // 将集合中的对象添加到此场景。
	This function adds the following types of objects to this scene: PxActor, PxAggregate, PxArticulation. 
	This method is typically used after deserializing the collection in order to populate the scene with deserialized objects.
           此函数向该场景添加以下类型的对象：PxActor、PxAggregate、PxArticulation。
           此方法通常在反序列化集合之后使用，以便使用反序列化的对象填充场景。
	\note If the collection contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
          如果集合包含具有无效约束的actor，则在检查构建中忽略调用并发出错误。
	\param[in] collection Objects to add to this scene. See #PxCollection
	@see PxCollection, PxConstraint::isValid()
	*/
	virtual	void				addCollection(const PxCollection& collection) = 0;
	//@}
	/************************************************************************************************/

	/** @name Contained Object Retrieval    // 获取包含对象
	*/
	//@{

	/**
	\brief Retrieve the number of actors of certain types in the scene.
	\param[in] types Combination of actor types.
	\return the number of actors.
	@see getActors()
	*/
	virtual	PxU32				getNbActors(PxActorTypeFlags types) const = 0;

	/**
	\brief Retrieve an array of all the actors of certain types in the scene.
	\param[in] types Combination of actor types to retrieve.
	\param[out] userBuffer The buffer to receive actor pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first actor pointer to be retrieved
	\return Number of actors written to the buffer.
	@see getNbActors()
	*/
	virtual	PxU32				getActors(PxActorTypeFlags types, PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	= 0;

	/**
	\brief Queries the PxScene for a list of the PxActors whose transforms have been updated during the previous simulation step
           查询 PxScene 中的 PxActor 列表，这些 PxActor 在上一个模拟步骤中其变换已更新
	\note PxSceneFlag::eENABLE_ACTIVE_ACTORS must be set.   必须设置 PxSceneFlag::eENABLE_ACTIVE_ACTORS。
	\note Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored and NULL will be returned.
          不要在模拟运行时使用此方法。 在模拟运行时调用此方法将被忽略并返回 NULL。
	\param[out] nbActorsOut The number of actors returned.
	\return A pointer to the list of active PxActors generated during the last call to fetchResults().
	@see PxActor
	*/
	virtual PxActor**		getActiveActors(PxU32& nbActorsOut) = 0;

	/**
	\brief Returns the number of articulations in the scene.
	\return the number of articulations in this scene.
	@see getArticulations()
	*/
	virtual PxU32				getNbArticulations() const = 0;

	/**
	\brief Retrieve all the articulations in the scene.
	\param[out] userBuffer The buffer to receive articulations pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first articulations pointer to be retrieved
	\return Number of articulations written to the buffer.
	@see getNbArticulations()
	*/
	virtual	PxU32				getArticulations(PxArticulationBase** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Returns the number of constraint shaders in the scene.   // 返回场景中约束着色器的数量。
	\return the number of constraint shaders in this scene.
	@see getConstraints()
	*/
	virtual PxU32				getNbConstraints()	const	= 0;

	/**
	\brief Retrieve all the constraint shaders in the scene.
	\param[out] userBuffer The buffer to receive constraint shader pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first constraint pointer to be retrieved
	\return Number of constraint shaders written to the buffer.
	@see getNbConstraints()
	*/
	virtual	PxU32				getConstraints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;


	/**
	\brief Returns the number of aggregates in the scene.
	\return the number of aggregates in this scene.
	@see getAggregates()
	*/
	virtual			PxU32		getNbAggregates()	const	= 0;

	/**
	\brief Retrieve all the aggregates in the scene.
	\param[out] userBuffer The buffer to receive aggregates pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first aggregate pointer to be retrieved
	\return Number of aggregates written to the buffer.
	@see getNbAggregates()
	*/
	virtual			PxU32		getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const	= 0;

	//@}
	/************************************************************************************************/

	/** @name Dominance                 // 支配
	*/
	//@{

	/**
	\brief Specifies the dominance behavior of contacts between two actors with two certain dominance groups.
           指定具有两个特定支配组的两个行动者之间接触的支配行为。
	It is possible to assign each actor to a dominance groups using #PxActor::setDominanceGroup().
    可以使用 #PxActor::setDominanceGroup() 将每个参与者分配到一个支配组。

	With dominance groups one can have all contacts created between actors act in one direction only. This is useful, for example, if you
	want an object to push debris out of its way and be unaffected,while still responding physically to forces and collisions
	with non-debris objects.
    对于支配群体，可以让参与者之间创建的所有联系都只向一个方向行事。 
    这很有用，例如，如果您希望一个对象将碎片推开并且不受影响，同时仍然对力和与非碎片对象的碰撞做出物理响应。

	Whenever a contact between two actors (a0, a1) needs to be solved, the groups (g0, g1) of both
	actors are retrieved. Then the PxDominanceGroupPair setting for this group pair is retrieved with getDominanceGroupPair(g0, g1).
    每当需要解决两个演员 (a0, a1) 之间的联系时，都会检索两个演员的组 (g0, g1)。 
    然后使用 getDominanceGroupPair(g0, g1) 检索该组对的 PxDominanceGroupPair 设置。

	In the contact, PxDominanceGroupPair::dominance0 becomes the dominance setting for a0, and 
	PxDominanceGroupPair::dominance1 becomes the dominance setting for a1. A dominanceN setting of 1.0f, the default, 
	will permit aN to be pushed or pulled by a(1-N) through the contact. A dominanceN setting of 0.0f, will however 
	prevent aN to be pushed by a(1-N) via the contact. Thus, a PxDominanceGroupPair of (1.0f, 0.0f) makes 
	the interaction one-way.
    在联系人中，PxDominanceGroupPair::dominance0 成为 a0 的优势设置，PxDominanceGroupPair::dominance1 成为 a1 的优势设置。 
    默认值 1.0f 的 dominanceN 设置将允许 a(1-N) 通过触点推动或拉动 aN。 
    然而，0.0f 的dominanceN 设置将阻止aN 通过触点被a(1-N) 推动。 因此，(1.0f, 0.0f) 的 xDominanceGroupPair 使交互单向。
	
	The matrix sampled by getDominanceGroupPair(g1, g2) is initialised by default such that:
    由 getDominanceGroupPair(g1, g2) 采样的矩阵默认初始化为：
	if g1 == g2, then (1.0f, 1.0f) is returned
	if g1 <  g2, then (0.0f, 1.0f) is returned
	if g1 >  g2, then (1.0f, 0.0f) is returned
	
	In other words, we permit actors in higher groups to be pushed around by actors in lower groups by default.
    换句话说，默认情况下，我们允许较高组的演员被较低组的演员推来推去。
		
	These settings should cover most applications, and in fact not overriding these settings may likely result in higher performance.
    这些设置应该涵盖大多数应用程序，实际上不覆盖这些设置可能会导致更高的性能。
	
	It is not possible to make the matrix asymetric, or to change the diagonal. In other words: 
    不可能使矩阵不对称，或改变对角线。 换句话说：
	* it is not possible to change (g1, g2) if (g1==g2)	    // 如果 (g1==g2) 则无法更改 (g1, g2)
	* if you set                                            // 如果你设置了
	
	(g1, g2) to X, then (g2, g1) will implicitly and automatically be set to ~X, where:
    (g1, g2) 到 X，那么 (g2, g1) 将隐式并自动设置为 ~X，其中:  
	~(1.0f, 1.0f) is (1.0f, 1.0f)
	~(0.0f, 1.0f) is (1.0f, 0.0f)
	~(1.0f, 0.0f) is (0.0f, 1.0f)
	
	These two restrictions are to make sure that contacts between two actors will always evaluate to the same dominance
	setting, regardless of the order of the actors.
    这两个限制是为了确保两个参与者之间的联系将始终评估为相同的支配设置，而不管参与者的顺序如何。  
	
	Dominance settings are currently specified as floats 0.0f or 1.0f because in the future we may permit arbitrary 
	fractional settings to express 'partly-one-way' interactions.
    优势设置当前指定为浮点数 0.0f 或 1.0f，因为将来我们可能允许任意分数设置来表达“部分单向”交互。
		
	<b>Sleeping:</b> Does <b>NOT</b> wake actors up automatically.  睡眠：不会自动唤醒actor。
	@see getDominanceGroupPair() PxDominanceGroup PxDominanceGroupPair PxActor::setDominanceGroup() PxActor::getDominanceGroup()
	*/
	virtual void				setDominanceGroupPair(
									PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance) = 0;

	/**
	\brief Samples the dominance matrix.
	@see setDominanceGroupPair() PxDominanceGroup PxDominanceGroupPair PxActor::setDominanceGroup() PxActor::getDominanceGroup()
	*/
	virtual PxDominanceGroupPair getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const = 0;

	//@}
	/************************************************************************************************/

	/** @name Dispatcher        调度员
	*/
	//@{

	/**
	\brief Return the cpu dispatcher that was set in PxSceneDesc::cpuDispatcher when creating the scene with PxPhysics::createScene
           返回使用 PxPhysics::createScene 创建场景时在 PxSceneDesc::cpuDispatcher 中设置的 CPU 调度程序
	@see PxSceneDesc::cpuDispatcher, PxPhysics::createScene
	*/
	virtual PxCpuDispatcher* getCpuDispatcher() const = 0;

	/**
	\brief Return the CUDA context manager that was set in PxSceneDesc::cudaContextManager when creating the scene with PxPhysics::createScene
           返回使用 PxPhysics::createScene 创建场景时在 PxSceneDesc::cudaContextManager 中设置的 CUDA 上下文管理器
	<b>Platform specific:</b> Applies to PC GPU only.   平台特定：仅适用于 PC GPU。
	@see PxSceneDesc::cudaContextManager, PxPhysics::createScene
	*/
	virtual PxCudaContextManager* getCudaContextManager() const = 0;

	//@}
	/************************************************************************************************/
	/** @name Multiclient           多客户端
	*/
	//@{
	/**
	\brief Reserves a new client ID.    保留一个新的客户端 ID。
	
	PX_DEFAULT_CLIENT is always available as the default clientID.
	Additional clients are returned by this function. Clients cannot be released once created. 
	An error is reported when more than a supported number of clients (currently 128) are created. 
    PX_DEFAULT_CLIENT 始终可用作默认客户端 ID。
    此函数返回其他客户端。 客户端一旦创建就无法释放。
    创建的客户端数量超过支持数量（当前为 128 个）时会报告错误。

	@see PxClientID
	*/
	virtual PxClientID			createClient() = 0;

	//@}

	/************************************************************************************************/

	/** @name Callbacks         回调
	*/
	//@{

	/**
	\brief Sets a user notify object which receives special simulation events when they occur.
           设置一个用户通知对象，当它们发生时接收特殊的模拟事件。
	\note Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
          不要在模拟运行时设置回调。 在模拟运行时调用此方法将被忽略。
	\param[in] callback User notification callback. See #PxSimulationEventCallback.
	@see PxSimulationEventCallback getSimulationEventCallback
	*/
	virtual void				setSimulationEventCallback(PxSimulationEventCallback* callback) = 0;

	/**
	\brief Retrieves the simulationEventCallback pointer set with setSimulationEventCallback().
           检索使用 setSimulationEventCallback() 设置的 simulationEventCallback 指针。
	\return The current user notify pointer. See #PxSimulationEventCallback.
	@see PxSimulationEventCallback setSimulationEventCallback()
	*/
	virtual PxSimulationEventCallback*	getSimulationEventCallback() const = 0;

	/**
	\brief Sets a user callback object, which receives callbacks on all contacts generated for specified actors.
           设置用户回调对象，该对象接收针对指定参与者生成的所有联系人的回调。
	\note Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
          不要在模拟运行时设置回调。 在模拟运行时调用此方法将被忽略。
	\param[in] callback Asynchronous user contact modification callback. See #PxContactModifyCallback.
	*/
	virtual void				setContactModifyCallback(PxContactModifyCallback* callback) = 0;

	/**
	\brief Sets a user callback object, which receives callbacks on all CCD contacts generated for specified actors.
           设置用户回调对象，该对象接收针对指定演员生成的所有 CCD 联系人的回调。
	\note Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
          不要在模拟运行时设置回调。 在模拟运行时调用此方法将被忽略。
	\param[in] callback Asynchronous user contact modification callback. See #PxCCDContactModifyCallback.
	*/
	virtual void				setCCDContactModifyCallback(PxCCDContactModifyCallback* callback) = 0;

	/**
	\brief Retrieves the PxContactModifyCallback pointer set with setContactModifyCallback().
	\return The current user contact modify callback pointer. See #PxContactModifyCallback.
	@see PxContactModifyCallback setContactModifyCallback()
	*/
	virtual PxContactModifyCallback*	getContactModifyCallback() const = 0;

	/**
	\brief Retrieves the PxCCDContactModifyCallback pointer set with setContactModifyCallback().
	\return The current user contact modify callback pointer. See #PxContactModifyCallback.
	@see PxContactModifyCallback setContactModifyCallback()
	*/
	virtual PxCCDContactModifyCallback*	getCCDContactModifyCallback() const = 0;

	/**
	\brief Sets a broad-phase user callback object. 设置一个广泛阶段的用户回调对象。
	\note Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
          不要在模拟运行时设置回调。 在模拟运行时调用此方法将被忽略。
	\param[in] callback	Asynchronous broad-phase callback. See #PxBroadPhaseCallback.
	*/
	virtual void				setBroadPhaseCallback(PxBroadPhaseCallback* callback) = 0;

	/**
	\brief Retrieves the PxBroadPhaseCallback pointer set with setBroadPhaseCallback().
	\return The current broad-phase callback pointer. See #PxBroadPhaseCallback.
	@see PxBroadPhaseCallback setBroadPhaseCallback()
	*/
	virtual PxBroadPhaseCallback* getBroadPhaseCallback()	const = 0;

	//@}
	/************************************************************************************************/

	/** @name Collision Filtering
	*/
	//@{

	/**
	\brief Sets the shared global filter data which will get passed into the filter shader.
           设置将传递到过滤器着色器的共享全局过滤器数据。
	\note It is the user's responsibility to ensure that changing the shared global filter data does not change the filter output value for existing pairs. 
	      If the filter output for existing pairs does change nonetheless then such a change will not take effect until the pair gets refiltered. 
		  resetFiltering() can be used to explicitly refilter the pairs of specific objects.
          用户有责任确保更改共享的全局过滤器数据不会更改现有对的过滤器输出值。
          如果现有对的过滤器输出确实发生了变化，那么这种更改将不会生效，直到对重新过滤。
          resetFiltering() 可用于显式重新过滤特定对象对。
	\note The provided data will get copied to internal buffers and this copy will be used for filtering calls.
          提供的数据将被复制到内部缓冲区，该副本将用于过滤调用。
	\note Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
          不要在模拟运行时使用此方法。 在模拟运行时调用此方法将被忽略。
	\param[in] data The shared global filter shader data.
	\param[in] dataSize Size of the shared global filter shader data (in bytes).
	@see getFilterShaderData() PxSceneDesc.filterShaderData PxSimulationFilterShader
	*/
	virtual void				setFilterShaderData(const void* data, PxU32 dataSize) = 0;

	/**
	\brief Gets the shared global filter data in use for this scene.
           获取用于此场景的共享全局过滤器数据。
	\note The reference points to a copy of the original filter data specified in #PxSceneDesc.filterShaderData or provided by #setFilterShaderData().
          该引用指向在#PxSceneDesc.filterShaderData 中指定或由#setFilterShaderData() 提供的原始过滤器数据的副本。
	\return Shared filter data for filter shader.
	@see getFilterShaderDataSize() setFilterShaderData() PxSceneDesc.filterShaderData PxSimulationFilterShader
	*/
	virtual	const void*			getFilterShaderData() const = 0;

	/**
	\brief Gets the size of the shared global filter data (#PxSceneDesc.filterShaderData)
           获取共享全局过滤器数据的大小（#PxSceneDesc.filterShaderData）
	\return Size of shared filter data [bytes].
	@see getFilterShaderData() PxSceneDesc.filterShaderDataSize PxSimulationFilterShader
	*/
	virtual	PxU32				getFilterShaderDataSize() const = 0;

	/**
	\brief Gets the custom collision filter shader in use for this scene.
	\return Filter shader class that defines the collision pair filtering.
	@see PxSceneDesc.filterShader PxSimulationFilterShader
	*/
	virtual	PxSimulationFilterShader	getFilterShader() const = 0;

	/**
	\brief Gets the custom collision filter callback in use for this scene.
	\return Filter callback class that defines the collision pair filtering.
	@see PxSceneDesc.filterCallback PxSimulationFilterCallback
	*/
	virtual	PxSimulationFilterCallback*	getFilterCallback() const = 0;

	/**
	\brief Marks the object to reset interactions and re-run collision filters in the next simulation step.
           标记对象以在下一个模拟步骤中重置交互并重新运行碰撞过滤器。
	This call forces the object to remove all existing collision interactions, to search anew for existing contact
	pairs and to run the collision filters again for found collision pairs.
    此调用强制对象删除所有现有的碰撞交互，重新搜索现有的接触对并再次运行碰撞过滤器以查找找到的碰撞对。

	\note The operation is supported for PxRigidActor objects only. 该操作仅支持 PxRigidActor 对象。

	\note All persistent state of existing interactions will be lost and can not be retrieved even if the same collison pair
	is found again in the next step. This will mean, for example, that you will not get notified about persistent contact
	for such an interaction (see #PxPairFlag::eNOTIFY_TOUCH_PERSISTS), the contact pair will be interpreted as newly found instead.
    即使在下一步中再次找到相同的碰撞对，现有交互的所有持久状态都将丢失并且无法恢复。 
    例如，这意味着您不会收到有关此类交互的持久联系的通知（请参阅#PxPairFlag::eNOTIFY_TOUCH_PERSISTS），该联系对将被解释为新发现的。

	\note Lost touch contact reports will be sent for every collision pair which includes this shape, if they have
	been requested through #PxPairFlag::eNOTIFY_TOUCH_LOST or #PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST.
    如果已通过#PxPairFlag::eNOTIFY_TOUCH_LOST 或#PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST 请求，将为包含此形状的每个碰撞对发送丢失触摸接触报告。

	\note This is an expensive operation, don't use it if you don't have to.
          这是一项昂贵的操作，如果您没有必要，请不要使用它。
	\note Can be used to retrieve collision pairs that were killed by the collision filters (see #PxFilterFlag::eKILL)
          可用于检索被碰撞过滤器杀死的碰撞对（参见#PxFilterFlag::eKILL）
	\note It is invalid to use this method if the actor has not been added to a scene already.
          如果演员尚未添加到场景中，则使用此方法无效。
	\note It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
          如果设置了 PxActorFlag::eDISABLE_SIMULATION，则使用此方法无效。
	<b>Sleeping:</b> Does wake up the actor.
	\param[in] actor The actor for which to re-evaluate interactions.
	@see PxSimulationFilterShader PxSimulationFilterCallback
	*/
	virtual void				resetFiltering(PxActor& actor) = 0;

	/**
	\brief Marks the object to reset interactions and re-run collision filters for specified shapes in the next simulation step.
           标记对象以在下一个模拟步骤中重置交互并重新运行指定形状的碰撞过滤器。
           This is a specialization of the resetFiltering(PxActor& actor) method and allows to reset interactions for specific shapes of a PxRigidActor.
           这是 resetFiltering(PxActor&actor) 方法的特化，允许为 PxRigidActor 的特定形状重置交互。
	<b>Sleeping:</b> Does wake up the actor.
	\param[in] actor The actor for which to re-evaluate interactions.
	\param[in] shapes The shapes for which to re-evaluate interactions.
	\param[in] shapeCount Number of shapes in the list.
	@see PxSimulationFilterShader PxSimulationFilterCallback
	*/
	virtual void				resetFiltering(PxRigidActor& actor, PxShape*const* shapes, PxU32 shapeCount) = 0;

	/**
	\brief Gets the pair filtering mode for kinematic-kinematic pairs.
           获取运动学-运动学对的对过滤模式。
	\return Filtering mode for kinematic-kinematic pairs.
	@see PxPairFilteringMode PxSceneDesc
	*/
	virtual	PxPairFilteringMode::Enum	getKinematicKinematicFilteringMode()	const	= 0;

	/**
	\brief Gets the pair filtering mode for static-kinematic pairs.
           获取静态-运动学对的对过滤模式。
	\return Filtering mode for static-kinematic pairs.
	@see PxPairFilteringMode PxSceneDesc
	*/
	virtual	PxPairFilteringMode::Enum	getStaticKinematicFilteringMode()		const	= 0;

	//@}
	/************************************************************************************************/

	/** @name Simulation            模拟
	*/
	//@{
	/**
 	\brief Advances the simulation by an elapsedTime time.
           按 elapsedTime 时间推进模拟。
	\note Large elapsedTime values can lead to instabilities. In such cases elapsedTime
	should be subdivided into smaller time intervals and simulate() should be called
	multiple times for each interval.
          较大的 elapsedTime 值会导致不稳定。 
          在这种情况下， elapsedTime 应该细分为更小的时间间隔，并且应该为每个间隔多次调用模拟（）。

	Calls to simulate() should pair with calls to fetchResults():
 	Each fetchResults() invocation corresponds to exactly one simulate()
 	invocation; calling simulate() twice without an intervening fetchResults()
 	or fetchResults() twice without an intervening simulate() causes an error
 	condition.
    对simulate() 的调用应该与对 fetchResults() 的调用配对 ：
    每个 fetchResults() 调用都对应一个simulate() 调用； 
    在不介入 fetchResults() 的情况下调用simulate() 两次或在不介入simulate() 的情况下调用 fetchResults() 两次会导致错误条件。

 	scene->simulate();
 	...do some processing until physics is computed...
 	scene->fetchResults();
 	...now results of run may be retrieved.

	\param[in] elapsedTime Amount of time to advance simulation by. The parameter has to be larger than 0, else the resulting behavior will be undefined. <b>Range:</b> (0, PX_MAX_F32)
	\param[in] completionTask if non-NULL, this task will have its refcount incremented in simulate(), then
	decremented when the scene is ready to have fetchResults called. So the task will not run until the
	application also calls removeReference().
	\param[in] scratchMemBlock a memory region for physx to use for temporary data during simulation. This block may be reused by the application
	after fetchResults returns. Must be aligned on a 16-byte boundary
	\param[in] scratchMemBlockSize the size of the scratch memory block. Must be a multiple of 16K.
	\param[in] controlSimulation if true, the scene controls its PxTaskManager simulation state. Leave
    true unless the application is calling the PxTaskManager start/stopSimulation() methods itself.

	@see fetchResults() checkResults()
	*/
	virtual	void				simulate(PxReal elapsedTime, physx::PxBaseTask* completionTask = NULL,
									void* scratchMemBlock = 0, PxU32 scratchMemBlockSize = 0, bool controlSimulation = true) = 0;


	/**
 	\brief Performs dynamics phase of the simulation pipeline.
           执行模拟管道的动力学阶段。
	\note Calls to advance() should follow calls to fetchCollision(). An error message will be issued if this sequence is not followed.
          对 Advance() 的调用应该跟在对 fetchCollision() 的调用之后。 如果不遵循此顺序，将发出错误消息。
	\param[in] completionTask if non-NULL, this task will have its refcount incremented in advance(), then
	decremented when the scene is ready to have fetchResults called. So the task will not run until the
	application also calls removeReference().
    如果非 NULL，则此任务的 refcount 将在 Advance() 之前递增，然后在场景准备好调用 fetchResults 时递减。 
    因此，在应用程序也调用 removeReference() 之前，该任务不会运行。
	*/
	virtual	void				advance(physx::PxBaseTask* completionTask = 0) = 0;

	/**
	\brief Performs collision detection for the scene over elapsedTime
           对经过 elapsedTime 的场景执行碰撞检测
	\note Calls to collide() should be the first method called to simulate a frame.
          调用collide() 应该是模拟帧的第一个方法。

	\param[in] elapsedTime Amount of time to advance simulation by. The parameter has to be larger than 0, else the resulting behavior will be undefined. <b>Range:</b> (0, PX_MAX_F32)
                推进模拟的时间量。 参数必须大于 0，否则结果行为将是未定义的。 范围：（0，PX_MAX_F32）
	\param[in] completionTask if non-NULL, this task will have its refcount incremented in collide(), then
	decremented when the scene is ready to have fetchResults called. So the task will not run until the
	application also calls removeReference().
                如果非 NULL，则此任务的引用计数将在 collide() 中递增，然后在场景准备好调用 fetchResults 时递减。 
                因此，在应用程序也调用 removeReference() 之前，该任务不会运行。
	\param[in] scratchMemBlock a memory region for physx to use for temporary data during simulation. This block may be reused by the application
	after fetchResults returns. Must be aligned on a 16-byte boundary
                physx 在模拟期间用于临时数据的内存区域。 在 fetchResults 返回后，该块可以被应用程序重用。 
                必须在 16 字节边界上对齐
	\param[in] scratchMemBlockSize the size of the scratch memory block. Must be a multiple of 16K.
                暂存内存块的大小。 必须是 16K 的倍数。
	\param[in] controlSimulation if true, the scene controls its PxTaskManager simulation state. Leave
    true unless the application is calling the PxTaskManager start/stopSimulation() methods itself.
                如果为 true，则场景控制其 PxTaskManager 模拟状态。 
                除非应用程序自己调用 PxTaskManager start/stopSimulation() 方法，否则保留为 true。
	*/
	virtual	void				collide(PxReal elapsedTime, physx::PxBaseTask* completionTask = 0, void* scratchMemBlock = 0,
									PxU32 scratchMemBlockSize = 0, bool controlSimulation = true) = 0;  
	
	/**
	\brief This checks to see if the simulation run has completed.
           这将检查模拟运行是否已完成。
	This does not cause the data available for reading to be updated with the results of the simulation, it is simply a status check.
	The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    这不会导致可供读取的数据随着模拟结果更新，它只是一个状态检查。
    bool 将允许它立即返回或阻塞等待条件满足，以便它可以返回 true
	\param[in] block When set to true will block until the condition is met.
                     当设置为 true 时将阻塞，直到满足条件。
	\return True if the results are available.
	@see simulate() fetchResults()
	*/
	virtual	bool				checkResults(bool block = false) = 0;

	/**
	This method must be called after collide() and before advance(). It will wait for the collision phase to finish. If the user makes an illegal simulation call, the SDK will issue an error message.
    这个方法必须在collide()之后和advance()之前调用。 它将等待碰撞阶段完成。 如果用户进行非法模拟调用，SDK 会发出错误信息。
	\param[in] block When set to true will block until the condition is met, which is collision must finish running.
                     当设置为 true 时将阻塞直到满足条件，即碰撞必须完成运行。
	*/
	virtual	bool				fetchCollision(bool block = false)	= 0;			

	/**
	This is the big brother to checkResults() it basically does the following:
    这是 checkResults() 的大哥，它基本上执行以下操作：
    \code
	    if ( checkResults(block) )
	    {
	    	fire appropriate callbacks
	    	swap buffers
	    	return true
	    }
	    else
	    	return false
	\endcode
	\param[in] block When set to true will block until results are available.
	\param[out] errorState Used to retrieve hardware error codes. A non zero value indicates an error.
	\return True if the results have been fetched.
	@see simulate() checkResults()
	*/
	virtual	bool				fetchResults(bool block = false, PxU32* errorState = 0)	= 0;


	/**
	This call performs the first section of fetchResults (callbacks fired before swapBuffers), and returns a pointer to a 
	to the contact streams output by the simulation. It can be used to process contact pairs in parallel, which is often a limiting factor
	for fetchResults() performance. 
    此调用执行 fetchResults 的第一部分（在 swapBuffers 之前触发的回调），并返回一个指向模拟输出的联系流的指针。 
    它可用于并行处理接触对，这通常是 fetchResults() 性能的限制因素。

	After calling this function and processing the contact streams, call fetchResultsFinish(). Note that writes to the simulation are not
	permitted between the start of fetchResultsStart() and the end of fetchResultsFinish().
    调用此函数并处理联系流后，调用 fetchResultsFinish()。 
    请注意，在 fetchResultsStart() 开始和 fetchResultsFinish() 结束之间不允许写入模拟。

	\param[in] block When set to true will block until results are available.
	\param[out] contactPairs an array of pointers to contact pair headers
	\param[out] nbContactPairs the number of contact pairs
	\return True if the results have been fetched.

	@see simulate() checkResults() fetchResults() fetchResultsFinish()
	*/
	virtual	bool				fetchResultsStart(const PxContactPairHeader*& contactPairs, PxU32& nbContactPairs, bool block = false) = 0;


	/**
	This call processes all event callbacks in parallel. It takes a continuation task, which will be executed once all callbacks have been processed.
    此调用并行处理所有事件回调。 它需要一个延续任务，一旦处理完所有回调，就会执行该任务。
	This is a utility function to make it easier to process callbacks in parallel using the PhysX task system. It can only be used in conjunction with 
	fetchResultsStart(...) and fetchResultsFinish(...)
    这是一个实用函数，可以更轻松地使用 PhysX 任务系统并行处理回调。 
    它只能与 fetchResultsStart(...) 和 fetchResultsFinish(...) 结合使用。
	\param[in] continuation The task that will be executed once all callbacks have been processed.
                            处理完所有回调后将执行的任务。
	*/
	virtual void				processCallbacks(physx::PxBaseTask* continuation) = 0;


	/**
	This call performs the second section of fetchResults: the buffer swap and subsequent callbacks.
    此调用执行 fetchResults 的第二部分：缓冲区交换和后续回调。
	It must be called after fetchResultsStart() returns and contact reports have been processed.
    必须在 fetchResultsStart() 返回并处理联系报告后调用它。
	Note that once fetchResultsFinish() has been called, the contact streams returned in fetchResultsStart() will be invalid.
    请注意，一旦调用了 fetchResultsFinish()，在 fetchResultsStart() 中返回的联系流将无效。
	\param[out] errorState Used to retrieve hardware error codes. A non zero value indicates an error.
	@see simulate() checkResults() fetchResults() fetchResultsStart()
	*/
	virtual	void				fetchResultsFinish(PxU32* errorState = 0) = 0;


	/**
	\brief Clear internal buffers and free memory.
           清除内部缓冲区并释放内存。
	This method can be used to clear buffers and free internal memory without having to destroy the scene. Can be useful if
	the physics data gets streamed in and a checkpoint with a clean state should be created.
    此方法可用于清除缓冲区和释放内部存储器，而无需破坏场景。 如果物理数据流入并且应该创建具有干净状态的检查点，则可能很有用。
	\note It is not allowed to call this method while the simulation is running. The call will fail.
          不允许在模拟运行时调用此方法。 调用将失败。
	\param[in] sendPendingReports When set to true pending reports will be sent out before the buffers get cleaned up (for instance lost touch contact/trigger reports due to deleted objects).
                    当设置为 true 时，将在清理缓冲区之前发送待处理报告（例如，由于删除的对象而丢失的接触接触/触发报告）。
	*/
	virtual	void				flushSimulation(bool sendPendingReports = false) = 0;
	
	/**
	\brief Sets a constant gravity for the entire scene.
           为整个场景设置恒定的重力。
	<b>Sleeping:</b> Does <b>NOT</b> wake the actor up automatically.
	\param[in] vec A new gravity vector(e.g. PxVec3(0.0f,-9.8f,0.0f) ) <b>Range:</b> force vector
	@see PxSceneDesc.gravity getGravity()
	*/
	virtual void				setGravity(const PxVec3& vec) = 0;

	/**
	\brief Retrieves the current gravity setting.
	\return The current gravity for the scene.
	@see setGravity() PxSceneDesc.gravity
	*/
	virtual PxVec3				getGravity() const = 0;

	/**
	\brief Set the bounce threshold velocity.  Collision speeds below this threshold will not cause a bounce.
           设置弹跳阈值速度。 低于此阈值的碰撞速度不会导致反弹。
	@see PxSceneDesc::bounceThresholdVelocity, getBounceThresholdVelocity
	*/
	virtual void				setBounceThresholdVelocity(const PxReal t) = 0;

	/**
	\brief Return the bounce threshold velocity.
	@see PxSceneDesc.bounceThresholdVelocity, setBounceThresholdVelocity
	*/
	virtual PxReal				getBounceThresholdVelocity() const = 0;


	/**
	\brief Sets the maximum number of CCD passes
	\param[in] ccdMaxPasses Maximum number of CCD passes
	@see PxSceneDesc.ccdMaxPasses getCCDMaxPasses()
	*/
	virtual void				setCCDMaxPasses(PxU32 ccdMaxPasses) = 0;

	/**
	\brief Gets the maximum number of CCD passes.
	\return The maximum number of CCD passes.
	@see PxSceneDesc::ccdMaxPasses setCCDMaxPasses()
	*/
	virtual PxU32				getCCDMaxPasses() const = 0;	

	/**
	\brief Return the value of frictionOffsetThreshold that was set in PxSceneDesc when creating the scene with PxPhysics::createScene
           返回使用 PxPhysics::createScene 创建场景时在 PxSceneDesc 中设置的摩擦偏移阈值的值
	@see PxSceneDesc::frictionOffsetThreshold,  PxPhysics::createScene
	*/
	virtual PxReal				getFrictionOffsetThreshold() const = 0;

	/**
	\brief Set the friction model.          设置摩擦模型。
	\deprecated The friction type cannot be changed after the first simulate call so this function is deprecated. Set the friction type at scene creation time in PxSceneDesc.
                第一次模拟调用后不能更改摩擦类型，因此不推荐使用此功能。 在 PxSceneDesc 中设置场景创建时的摩擦类型。
	@see PxFrictionType, PxSceneDesc::frictionType
	*/
	PX_DEPRECATED	virtual void setFrictionType(PxFrictionType::Enum frictionType) = 0;

	/**
	\brief Return the friction model.
	@see PxFrictionType, PxSceneDesc::frictionType
	*/
	virtual PxFrictionType::Enum getFrictionType() const = 0;

	//@}
	/************************************************************************************************/

	/** @name Visualization and Statistics      可视化和统计
	*/
	//@{
	/**
	\brief Function that lets you set debug visualization parameters.
           可让您设置调试可视化参数的函数。
	Returns false if the value passed is out of range for usage specified by the enum.
    如果传递的值超出枚举指定的使用范围，则返回 false。
	\param[in] param	Parameter to set. See #PxVisualizationParameter
	\param[in] value	The value to set, see #PxVisualizationParameter for allowable values. Setting to zero disables visualization for the specified property, setting to a positive value usually enables visualization and defines the scale factor.
	\return False if the parameter is out of range.
	@see getVisualizationParameter PxVisualizationParameter getRenderBuffer()
	*/
	virtual bool				setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value) = 0;

	/**
	\brief Function that lets you query debug visualization parameters.
           可让您查询调试可视化参数的函数。
	\param[in] paramEnum The Parameter to retrieve.
	\return The value of the parameter.
	@see setVisualizationParameter PxVisualizationParameter
	*/
	virtual PxReal				getVisualizationParameter(PxVisualizationParameter::Enum paramEnum) const = 0;


	/**
	\brief Defines a box in world space to which visualization geometry will be (conservatively) culled. Use a non-empty culling box to enable the feature, and an empty culling box to disable it.
           在世界空间中定义一个框，可视化几何体将（保守地）剔除到该框。 使用非空剔除框启用该功能，使用空剔除框禁用它。
	\param[in] box the box to which the geometry will be culled. Empty box to disable the feature.
	@see setVisualizationParameter getVisualizationCullingBox getRenderBuffer()
	*/
	virtual void				setVisualizationCullingBox(const PxBounds3& box) = 0;

	/**
	\brief Retrieves the visualization culling box.
           检索可视化剔除框。
	\return the box to which the geometry will be culled.
            几何体将被剔除到的框。
	@see setVisualizationParameter setVisualizationCullingBox 
	*/
	virtual PxBounds3			getVisualizationCullingBox() const = 0;
	
	/**
	\brief Retrieves the render buffer.
           检索渲染缓冲区。
	This will contain the results of any active visualization for this scene.
    这将包含此场景的任何活动可视化的结果。
	\note Do not use this method while the simulation is running. Calls to this method while result in undefined behaviour.
          不要在模拟运行时使用此方法。 调用此方法会导致未定义的行为。
	\return The render buffer.
	@see PxRenderBuffer
	*/
	virtual const PxRenderBuffer& getRenderBuffer() = 0;
	
	/**
	\brief Call this method to retrieve statistics for the current simulation step.
           调用此方法以检索当前模拟步骤的统计信息。
	\note Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
          不要在模拟运行时使用此方法。 在模拟运行时调用此方法将被忽略。
	\param[out] stats Used to retrieve statistics for the current simulation step.
                      用于检索当前模拟步骤的统计信息。
	@see PxSimulationStatistics
	*/
	virtual	void				getSimulationStatistics(PxSimulationStatistics& stats) const = 0;
	
	
	//@}
	/************************************************************************************************/

	/** @name Scene Query
	*/
	//@{

	/**
	\brief Return the value of PxSceneDesc::staticStructure that was set when creating the scene with PxPhysics::createScene
           返回使用 PxPhysics::createScene 创建场景时设置的 PxSceneDesc::staticStructure 的值
	@see PxSceneDesc::staticStructure, PxPhysics::createScene
	*/
	virtual	PxPruningStructureType::Enum getStaticStructure() const = 0;

	/**
	\brief Return the value of PxSceneDesc::dynamicStructure that was set when creating the scene with PxPhysics::createScene
           返回使用 PxPhysics::createScene 创建场景时设置的 PxSceneDesc::dynamicStructure 的值
	@see PxSceneDesc::dynamicStructure, PxPhysics::createScene
	*/
	virtual PxPruningStructureType::Enum getDynamicStructure() const = 0;

	/**
	\brief Flushes any changes to the scene query representation.
           刷新对场景查询表示的任何更改。
	This method updates the state of the scene query representation to match changes in the scene state.
    此方法更新场景查询表示的状态以匹配场景状态的变化。
	By default, these changes are buffered until the next query is submitted. Calling this function will not change
	the results from scene queries, but can be used to ensure that a query will not perform update work in the course of 
	its execution.
    默认情况下，这些更改会被缓冲，直到提交下一个查询。 
    调用此函数不会改变场景查询的结果，但可用于确保查询在其执行过程中不会执行更新工作。
	A thread performing updates will hold a write lock on the query structure, and thus stall other querying threads. In multithread
	scenarios it can be useful to explicitly schedule the period where this lock may be held for a significant period, so that
	subsequent queries issued from multiple threads will not block.
    执行更新的线程将持有查询结构的写锁，从而停止其他查询线程。 
    在多线程场景中，显式安排可能持有此锁的时间段可能会很有用，这样从多个线程发出的后续查询将不会阻塞。
	*/
	virtual	void				flushQueryUpdates() = 0;

	/**
	\brief Creates a BatchQuery object.     创建一个 BatchQuery 对象。
	Scene queries like raycasts, overlap tests and sweeps are batched in this object and are then executed at once. See #PxBatchQuery.
    光线投射、重叠测试和扫描等场景查询在此对象中进行批处理，然后立即执行。 请参阅#PxBatchQuery。
	\deprecated The batched query feature has been deprecated in PhysX version 3.4
                PhysX 3.4 版本中已弃用批量查询功能
	\param[in] desc The descriptor of scene query. Scene Queries need to register a callback. See #PxBatchQueryDesc.
                    场景查询的描述符。 场景查询需要注册回调。 请参阅#PxBatchQueryDesc。
	@see PxBatchQuery PxBatchQueryDesc
	*/
	PX_DEPRECATED virtual	PxBatchQuery*		createBatchQuery(const PxBatchQueryDesc& desc) = 0;

	/**
	\brief Sets the rebuild rate of the dynamic tree pruning structures.
           设置动态树修剪结构的重建率。
	\param[in] dynamicTreeRebuildRateHint Rebuild rate of the dynamic tree pruning structures.
                动态树修剪结构的重建率。
	@see PxSceneDesc.dynamicTreeRebuildRateHint getDynamicTreeRebuildRateHint() forceDynamicTreeRebuild()
	*/
	virtual	void				setDynamicTreeRebuildRateHint(PxU32 dynamicTreeRebuildRateHint) = 0;

	/**
	\brief Retrieves the rebuild rate of the dynamic tree pruning structures.
	\return The rebuild rate of the dynamic tree pruning structures.
	@see PxSceneDesc.dynamicTreeRebuildRateHint setDynamicTreeRebuildRateHint() forceDynamicTreeRebuild()
	*/
	virtual PxU32				getDynamicTreeRebuildRateHint() const = 0;

	/**
	\brief Forces dynamic trees to be immediately rebuilt.
           强制立即重建动态树。
	\param[in] rebuildStaticStructure	True to rebuild the dynamic tree containing static objects
                    True 重建包含静态对象的动态树
	\param[in] rebuildDynamicStructure	True to rebuild the dynamic tree containing dynamic objects
	@see PxSceneDesc.dynamicTreeRebuildRateHint setDynamicTreeRebuildRateHint() getDynamicTreeRebuildRateHint()
	*/
	virtual void				forceDynamicTreeRebuild(bool rebuildStaticStructure, bool rebuildDynamicStructure)	= 0;

	/**
	\brief Sets scene query update mode	        设置场景查询更新模式
	\param[in] updateMode	Scene query update mode.
	@see PxSceneQueryUpdateMode::Enum
	*/
	virtual void				setSceneQueryUpdateMode(PxSceneQueryUpdateMode::Enum updateMode) = 0;

	/**
	\brief Gets scene query update mode	
	\return Current scene query update mode.
	@see PxSceneQueryUpdateMode::Enum
	*/
	virtual PxSceneQueryUpdateMode::Enum getSceneQueryUpdateMode() const = 0;

	/**
	\brief Executes scene queries update tasks. 执行场景查询更新任务。
	This function will refit dirty shapes within the pruner and will execute a task to build a new AABB tree, which is
	build on a different thread. The new AABB tree is built based on the dynamic tree rebuild hint rate. Once
	the new tree is ready it will be commited in next fetchQueries call, which must be called after.
    该函数将在 pruner 中重新调整脏形状，并将执行一个任务来构建一个新的 AABB 树，该树构建在不同的线程上。 
    新的 AABB 树是基于动态树重建提示率构建的。 一旦新树准备好，它将在下一次 fetchQueries 调用中提交，该调用必须在之后调用。
	\note If PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED is used, it is required to update the scene queries
	using this function.
          如果使用 PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED，则需要使用该函数更新场景查询
	\param[in] completionTask if non-NULL, this task will have its refcount incremented in sceneQueryUpdate(), then
	decremented when the scene is ready to have fetchQueries called. So the task will not run until the
	application also calls removeReference().
                如果非 NULL，则此任务的引用计数将在 sceneQueryUpdate() 中递增，然后在场景准备好调用 fetchQueries 时递减。 
                因此，在应用程序也调用 removeReference() 之前，该任务不会运行。
	\param[in] controlSimulation if true, the scene controls its PxTaskManager simulation state. Leave
    true unless the application is calling the PxTaskManager start/stopSimulation() methods itself.
                如果为 true，则场景控制其 PxTaskManager 模拟状态。 
                除非应用程序自己调用 PxTaskManager start/stopSimulation() 方法，否则保留为 true。
	@see PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED
	*/
	virtual void				sceneQueriesUpdate(physx::PxBaseTask* completionTask = NULL, bool controlSimulation = true)	= 0;

	/**
	\brief This checks to see if the scene queries update has completed.
           这将检查场景查询更新是否已完成。
	This does not cause the data available for reading to be updated with the results of the scene queries update, it is simply a status check.
	The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    这不会导致可供读取的数据随着场景查询更新的结果而更新，它只是一个状态检查。
    bool 将允许它立即返回或阻塞等待条件满足，以便它可以返回 true。
	\param[in] block When set to true will block until the condition is met.
	\return True if the results are available.
	@see sceneQueriesUpdate() fetchResults()
	*/
	virtual	bool				checkQueries(bool block = false) = 0;

	/**
	This method must be called after sceneQueriesUpdate. It will wait for the scene queries update to finish. If the user makes an illegal scene queries update call, 
	the SDK will issue an error	message.
    这个方法必须在sceneQueriesUpdate 之后调用。 它将等待场景查询更新完成。 如果用户进行非法场景查询更新调用，SDK 会报错。
	If a new AABB tree build finished, then during fetchQueries the current tree within the pruning structure is swapped with the new tree. 
    如果新的 AABB 树构建完成，则在 fetchQueries 期间，修剪结构中的当前树将与新树交换。
	\param[in] block When set to true will block until the condition is met, which is tree built task must finish running.
                     当设置为 true 时将阻塞直到满足条件，即树构建任务必须完成运行。
	*/
	virtual	bool				fetchQueries(bool block = false)	= 0;	

	/**
	\brief Performs a raycast against objects in the scene, returns results in a PxRaycastBuffer object
	or via a custom user callback implementation inheriting from PxRaycastCallback.
           对场景中的对象执行光线投射，在 PxRaycastBuffer 对象中或通过从 PxRaycastCallback 继承的自定义用户回调实现返回结果。
	\note	Touching hits are not ordered.  触摸命中不排序。
	\note	Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in user guide article SceneQuery. User can ignore such objects by employing one of the provided filter mechanisms.
            根据形状类型，从对象内部发射光线会导致不同的结果。 
            请查看用户指南文章 SceneQuery 中的详细信息。 用户可以通过使用提供的过滤机制之一来忽略此类对象。
	\param[in] origin		Origin of the ray.
	\param[in] unitDir		Normalized direction of the ray.
	\param[in] distance		Length of the ray. Has to be in the [0, inf) range.
	\param[out] hitCall		Raycast hit buffer or callback object used to report raycast hits.
	\param[in] hitFlags		Specifies which properties per hit should be computed and returned via the hit callback.
	\param[in] filterData	Filtering data passed to the filter shader. See #PxQueryFilterData #PxBatchQueryPreFilterShader, #PxBatchQueryPostFilterShader
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxQueryFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Ray is tested against cached shape first. If no hit is found the ray gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\return True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
            如果发现任何接触或阻挡命中，或者在指定 PxQueryFlag::eANY_HIT 的情况下发现任何命中，则为真。
	@see PxRaycastCallback PxRaycastBuffer PxQueryFilterData PxQueryFilterCallback PxQueryCache PxRaycastHit PxQueryFlag PxQueryFlag::eANY_HIT
	*/
	virtual bool				raycast(
									const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
									PxRaycastCallback& hitCall, PxHitFlags hitFlags = PxHitFlags(PxHitFlag::eDEFAULT),
									const PxQueryFilterData& filterData = PxQueryFilterData(), PxQueryFilterCallback* filterCall = NULL,
									const PxQueryCache* cache = NULL) const = 0;

	/**
	\brief Performs a sweep test against objects in the scene, returns results in a PxSweepBuffer object
	or via a custom user callback implementation inheriting from PxSweepCallback.
        对场景中的对象执行扫描测试，在 PxSweepBuffer 对象中或通过从 PxSweepCallback 继承的自定义用户回调实现返回结果。
	\note	Touching hits are not ordered.  触摸命中不排序。
	\note	If a shape from the scene is already overlapping with the query shape in its starting position,
			the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
        如果场景中的形状已经与其起始位置的查询形状重叠，则除非指定了 eASSUME_NO_INITIAL_OVERLAP，否则将返回命中。
	\param[in] geometry		Geometry of object to sweep (supported types are: box, sphere, capsule, convex).
	\param[in] pose			Pose of the sweep object.
	\param[in] unitDir		Normalized direction of the sweep.
	\param[in] distance		Sweep distance. Needs to be in [0, inf) range and >0 if eASSUME_NO_INITIAL_OVERLAP was specified. Will be clamped to PX_MAX_SWEEP_DISTANCE.
	\param[out] hitCall		Sweep hit buffer or callback object used to report sweep hits.
	\param[in] hitFlags		Specifies which properties per hit should be computed and returned via the hit callback.
	\param[in] filterData	Filtering data and simple logic.
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxQueryFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Sweep is performed against cached shape first. If no hit is found the sweep gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\param[in] inflation	This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.
							Note: ePRECISE_SWEEP doesn't support inflation. Therefore the sweep will be performed with zero inflation.	
	\return True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
        如果发现任何接触或阻挡命中，或者在指定 PxQueryFlag::eANY_HIT 的情况下发现任何命中，则为真。
	@see PxSweepCallback PxSweepBuffer PxQueryFilterData PxQueryFilterCallback PxSweepHit PxQueryCache
	*/
	virtual bool				sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
									PxSweepCallback& hitCall, PxHitFlags hitFlags = PxHitFlags(PxHitFlag::eDEFAULT),
									const PxQueryFilterData& filterData = PxQueryFilterData(), PxQueryFilterCallback* filterCall = NULL,
									const PxQueryCache* cache = NULL, const PxReal inflation = 0.f) const = 0;


	/**
	\brief Performs an overlap test of a given geometry against objects in the scene, returns results in a PxOverlapBuffer object
	or via a custom user callback implementation inheriting from PxOverlapCallback.
        针对场景中的对象执行给定几何体的重叠测试，在 PxOverlapBuffer 对象中或通过从 PxOverlapCallback 继承的自定义用户回调实现返回结果。
	\note Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see #PxQueryHitType).
        过滤：从重叠查询的用户过滤器返回 eBLOCK 将导致警告（请参阅#PxQueryHitType）。
	\param[in] geometry		Geometry of object to check for overlap (supported types are: box, sphere, capsule, convex).
	\param[in] pose			Pose of the object.
	\param[out] hitCall		Overlap hit buffer or callback object used to report overlap hits.
	\param[in] filterData	Filtering data and simple logic. See #PxQueryFilterData #PxQueryFilterCallback
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxQueryFlag flags are set. If NULL, all hits are assumed to overlap.
        自定义过滤逻辑（可选）。 仅在设置了相应的 #PxQueryFlag 标志时使用。 如果为 NULL，则假定所有命中都重叠。
	\return True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
        如果发现任何接触或阻挡命中，或者在指定 PxQueryFlag::eANY_HIT 的情况下发现任何命中，则为真。
	\note eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
        eBLOCK 不应从用于overlap()的用户过滤器返回。 这样做将导致未定义的行为，并会发出警告。
	\note If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.
        如果设置了 PxQueryFlag::eNO_BLOCK 标志，则 eBLOCK 将自动转换为 eTOUCH 并抑制警告。
	@see PxOverlapCallback PxOverlapBuffer PxHitFlags PxQueryFilterData PxQueryFilterCallback
	*/
	virtual bool	overlap(const PxGeometry& geometry, const PxTransform& pose, PxOverlapCallback& hitCall,
					        const PxQueryFilterData& filterData = PxQueryFilterData(), 
                            PxQueryFilterCallback* filterCall = NULL
					       ) const = 0;


	/**
	\brief Retrieves the scene's internal scene query timestamp, increased each time a change to the
	static scene query structure is performed.
           检索场景的内部场景查询时间戳，每次更改静态场景查询结构时都会增加。
	\return scene query static timestamp
	*/
	virtual	PxU32	getSceneQueryStaticTimestamp()	const	= 0;
	//@}
	
	/************************************************************************************************/
	/** @name Broad-phase           粗略的碰撞检测。用空间划分的方式保存Bounding Volume，筛选出可能互相碰撞的刚体对。
	*/
	//@{

	/**
	\brief Returns broad-phase type.
	\return Broad-phase type
	*/
	virtual	PxBroadPhaseType::Enum	getBroadPhaseType()								const = 0;

	/**
	\brief Gets broad-phase caps.
	\param[out]	caps	Broad-phase caps
	\return True if success
	*/
	virtual	bool					getBroadPhaseCaps(PxBroadPhaseCaps& caps)			const = 0;

	/**
	\brief Returns number of regions currently registered in the broad-phase.
	\return Number of regions
	*/
	virtual	PxU32					getNbBroadPhaseRegions()							const = 0;

	/**
	\brief Gets broad-phase regions.
	\param[out]	userBuffer	Returned broad-phase regions
	\param[in]	bufferSize	Size of userBuffer
	\param[in]	startIndex	Index of first desired region, in [0 ; getNbRegions()[
	\return Number of written out regions
	*/
	virtual	PxU32					getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	= 0;

	/**
	\brief Adds a new broad-phase region. 

	Note that by default, objects already existing in the SDK that might touch this region will not be automatically
	added to the region. In other words the newly created region will be empty, and will only be populated with new
	objects when they are added to the simulation, or with already existing objects when they are updated.
    请注意，默认情况下，SDK 中已经存在的可能触及该区域的对象不会自动添加到该区域。 
    换句话说，新创建的区域将是空的，并且只会在将新对象添加到模拟时填充新对象，或者在更新时填充现有对象。

	It is nonetheless possible to override this default behavior and let the SDK populate the new region automatically
	with already existing objects overlapping the incoming region. This has a cost though, and it should only be used
	when the game can not guarantee that all objects within the new region will be added to the simulation after the
	region itself.
    尽管如此，还是可以覆盖此默认行为，并让 SDK 使用与传入区域重叠的现有对象自动填充新区域。 
    但这有成本，只有当游戏不能保证新区域内的所有对象都会在区域本身之后添加到模拟中时才应该使用它。

	\param[in]	region			User-provided region data
	\param[in]	populateRegion	Automatically populate new region with already existing objects overlapping it
	\return Handle for newly created region, or 0xffffffff in case of failure.
            处理新创建的区域，或在失败时为 0xffffffff。
	*/
	virtual	PxU32					addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion=false)		= 0;

	/**
	\brief Removes a new broad-phase region.

	If the region still contains objects, and if those objects do not overlap any region any more, they are not
	automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
	is used for each object. Users are responsible for removing the objects from the simulation if this is the
	desired behavior.
    如果该区域仍然包含对象，并且这些对象不再与任何区域重叠，则它们不会自动从模拟中删除。 
    相反，每个对象都使用 PxBroadPhaseCallback::onObjectOutOfBounds 通知。 如果这是所需的行为，用户有责任从模拟中删除对象。

	If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.
    如果句柄无效，或者有效句柄被删除两次，则会向错误流发送一条错误消息。

	\param[in]	handle	Region's handle, as returned by PxScene::addBroadPhaseRegion.
	\return True if success
	*/
	virtual	bool					removeBroadPhaseRegion(PxU32 handle)				= 0;

	//@}

	/************************************************************************************************/

	/** @name Threads and Memory        线程和内存
	*/
	//@{

	/**
	\brief Get the task manager associated with this scene
	\return the task manager associated with the scene
	*/
	virtual PxTaskManager*			getTaskManager() const = 0;


	/**
	\brief Lock the scene for reading from the calling thread.
           锁定场景以从调用线程读取。
	When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockRead() must be 
	called before any read calls are made on the scene.
    当启用 PxSceneFlag::eREQUIRE_RW_LOCK 标志时，必须在场景上进行任何读取调用之前调用 lockRead()。

	Multiple threads may read at the same time, no threads may read while a thread is writing.
	If a call to lockRead() is made while another thread is holding a write lock 
	then the calling thread will be blocked until the writing thread calls unlockWrite().
    多个线程可以同时读取，一个线程写入时没有线程可以读取。
    如果在另一个线程持有写锁时调用 lockRead()，则调用线程将被阻塞，直到写线程调用 unlockWrite()

	\note Lock upgrading is *not* supported, that means it is an error to call lockRead() followed by lockWrite().
          不支持锁升级，这意味着调用 lockRead() 后跟 lockWrite() 是错误的。
	\note Recursive locking is supported but each lockRead() call must be paired with an unlockRead().
          支持递归锁定，但每个 lockRead() 调用必须与一个 unlockRead() 配对。
	\param file String representing the calling file, for debug purposes
	\param line The source file line number, for debug purposes
	*/
	virtual void lockRead(const char* file=NULL, PxU32 line=0) = 0;

	/** 
	\brief Unlock the scene from reading.
	\note Each unlockRead() must be paired with a lockRead() from the same thread.
	*/
	virtual void unlockRead() = 0;

	/**
	\brief Lock the scene for writing from this thread.
           锁定场景以从此线程写入。
	When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockWrite() must be 
	called before any write calls are made on the scene.
    当启用 PxSceneFlag::eREQUIRE_RW_LOCK 标志时，必须在场景上进行任何写入调用之前调用 lockWrite()。

	Only one thread may write at a time and no threads may read while a thread is writing.
	If a call to lockWrite() is made and there are other threads reading then the 
	calling thread will be blocked until the readers complete.
    一次只能有一个线程写入，并且在一个线程写入时没有线程可以读取。
    如果调用 lockWrite() 并且有其他线程正在读取，则调用线程将被阻塞，直到读取器完成

	Writers have priority. If a thread is blocked waiting to write then subsequent calls to 
	lockRead() from other threads will be blocked until the writer completes.
    wirters有优先权。 如果一个线程在等待写入时被阻塞，则其他线程对 lockRead() 的后续调用将被阻塞，直到写入器完成。

	\note If multiple threads are waiting to write then the thread that is first granted access depends on OS scheduling.
          如果多个线程正在等待写入，那么首先被授予访问权限的线程取决于操作系统调度。
	\note Recursive locking is supported but each lockWrite() call must be paired with an unlockWrite().	
          支持递归锁定，但每个 lockWrite() 调用必须与一个 unlockWrite() 配对。
	\note If a thread has already locked the scene for writing then it may call
	lockRead().

	\param file String representing the calling file, for debug purposes
	\param line The source file line number, for debug purposes
	*/
	virtual void lockWrite(const char* file=NULL, PxU32 line=0) = 0;

	/**
	\brief Unlock the scene from writing.
	\note Each unlockWrite() must be paired with a lockWrite() from the same thread.
	*/
	virtual void unlockWrite() = 0;
	

	/**
	\brief set the cache blocks that can be used during simulate(). 
           设置可以在模拟（）期间使用的缓存块。
	Each frame the simulation requires memory to store contact, friction, and contact cache data. This memory is used in blocks of 16K.
	Each frame the blocks used by the previous frame are freed, and may be retrieved by the application using PxScene::flushSimulation()
    模拟的每一帧都需要内存来存储接触、摩擦和接触缓存数据。 该内存以 16K 的块为单位使用。
    每帧前一帧使用的块都被释放，并且可以由应用程序使用 PxScene::flushSimulation() 检索
	This call will force allocation of cache blocks if the numBlocks parameter is greater than the currently allocated number
	of blocks, and less than the max16KContactDataBlocks parameter specified at scene creation time.
    如果 numBlocks 参数大于当前分配的块数，并且小于在场景创建时指定的 max16KContactDataBlocks 参数，则此调用将强制分配缓存块。
	\param[in] numBlocks The number of blocks to allocate.	
	@see PxSceneDesc.nbContactDataBlocks PxSceneDesc.maxNbContactDataBlocks flushSimulation() getNbContactDataBlocksUsed getMaxNbContactDataBlocksUsed
	*/
	virtual         void				setNbContactDataBlocks(PxU32 numBlocks) = 0;
	

	/**
	\brief get the number of cache blocks currently used by the scene 
           获取场景当前使用的缓存块数
	This function may not be called while the scene is simulating
    场景模拟时可能不会调用此函数
	\return the number of cache blocks currently used by the scene
	@see PxSceneDesc.nbContactDataBlocks PxSceneDesc.maxNbContactDataBlocks flushSimulation() setNbContactDataBlocks() getMaxNbContactDataBlocksUsed()
	*/
	virtual         PxU32				getNbContactDataBlocksUsed() const = 0;

	/**
	\brief get the maximum number of cache blocks used by the scene 
           获取场景使用的最大缓存块数
	This function may not be called while the scene is simulating
    场景模拟时可能不会调用此函数
	\return the maximum number of cache blocks everused by the scene
            场景曾经使用的缓存块的最大数量
	@see PxSceneDesc.nbContactDataBlocks PxSceneDesc.maxNbContactDataBlocks flushSimulation() setNbContactDataBlocks() getNbContactDataBlocksUsed()
	*/
	virtual         PxU32				getMaxNbContactDataBlocksUsed() const = 0;


	/**
	\brief Return the value of PxSceneDesc::contactReportStreamBufferSize that was set when creating the scene with PxPhysics::createScene
           返回使用 PxPhysics::createScene 创建场景时设置的 PxSceneDesc::contactReportStreamBufferSize 的值
	@see PxSceneDesc::contactReportStreamBufferSize, PxPhysics::createScene
	*/
	virtual PxU32 getContactReportStreamBufferSize() const = 0;

	
	/**
	\brief Sets the number of actors required to spawn a separate rigid body solver thread.
           设置生成单独的刚体解算器线程所需的角色数。
	\param[in] solverBatchSize Number of actors required to spawn a separate rigid body solver thread.
	@see PxSceneDesc.solverBatchSize getSolverBatchSize()
	*/
	virtual	void						setSolverBatchSize(PxU32 solverBatchSize) = 0;

	/**
	\brief Retrieves the number of actors required to spawn a separate rigid body solver thread.
           检索生成单独的刚体解算器线程所需的角色数。
	\return Current number of actors required to spawn a separate rigid body solver thread.
            生成单独的刚体解算器线程所需的当前角色数。
	@see PxSceneDesc.solverBatchSize setSolverBatchSize()
	*/
	virtual PxU32						getSolverBatchSize() const = 0;

	/**
	\brief Sets the number of articulations required to spawn a separate rigid body solver thread.
           设置生成单独的刚体解算器线程所需的关节数。
	\param[in] solverBatchSize Number of articulations required to spawn a separate rigid body solver thread.
	@see PxSceneDesc.solverBatchSize getSolverArticulationBatchSize()
	*/
	virtual	void						setSolverArticulationBatchSize(PxU32 solverBatchSize) = 0;

	/**
	\brief Retrieves the number of articulations required to spawn a separate rigid body solver thread.
           检索生成单独的刚体解算器线程所需的关节数。
	\return Current number of articulations required to spawn a separate rigid body solver thread.
	@see PxSceneDesc.solverBatchSize setSolverArticulationBatchSize()
	*/
	virtual PxU32						getSolverArticulationBatchSize() const = 0;
	

	//@}

	/**
	\brief Returns the wake counter reset value.    返回唤醒计数器重置值。
	\return Wake counter reset value
	@see PxSceneDesc.wakeCounterResetValue
	*/
	virtual	PxReal						getWakeCounterResetValue() const = 0;

	/**
	\brief Shift the scene origin by the specified vector.
           将场景原点移动指定的向量。
	The poses of all objects in the scene and the corresponding data structures will get adjusted to reflect the new origin location
	(the shift vector will get subtracted from all object positions).
    场景中所有物体的姿态和相应的数据结构将得到调整以反映新的原点位置（位移矢量将从所有物体位置中减去）。
	\note It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysX accordingly.
          用户有责任跟踪总原点偏移并相应地调整到/来自 PhysX 的所有输入/输出。
	\note Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
          不要在模拟运行时使用此方法。 在模拟运行时调用此方法将被忽略。
	\note Make sure to propagate the origin shift to other dependent modules (for example, the character controller module etc.).
          确保将原点转移传播到其他相关模块（例如，角色控制器模块等）。
	\note This is an expensive operation and we recommend to use it only in the case where distance related precision issues may arise in areas far from the origin.
          这是一项昂贵的操作，我们建议仅在远离原点的区域可能出现与距离相关的精度问题的情况下使用它。
	\param[in] shift Translation vector to shift the origin by.
	*/
	virtual	void					shiftOrigin(const PxVec3& shift) = 0;

	/**
	\brief Returns the Pvd client associated with the scene.    返回与场景关联的 Pvd 客户端。
	\return the client, NULL if no PVD supported.
	*/
	virtual PxPvdSceneClient*		getScenePvdClient() = 0;

    // 用户可以将它分配给任何东西，通常是为了与用户对象创建 1:1 的关系。
	void*	userData;	//!< user can assign this to whatever, usually to create a 1:1 relationship with a user object.
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
