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


#ifndef PX_PHYSICS_NX_FILTERING
#define PX_PHYSICS_NX_FILTERING
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "foundation/PxFlags.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxActor;
class PxShape;

static const PxU32 INVALID_FILTER_PAIR_INDEX = 0xffffffff;

/**
\brief Collection of flags describing the actions to take for a collision pair.
@see PxPairFlags PxSimulationFilterShader.filter() PxSimulationFilterCallback
*/
struct PxPairFlag                   // 碰撞对的相关事件flag
{
	enum Enum
	{
		/**
		\brief Process the contacts of this collision pair in the dynamics solver. 在动态处理器中处理碰撞对的接触。  
		\note Only takes effect if the colliding actors are rigid bodies. 仅当碰撞体是刚体时才生效  
		*/
		eSOLVE_CONTACT						= (1<<0),

		/**
		\brief Call contact modification callback for this collision pair. 调用碰撞对的接触modify回调
		\note Only takes effect if the colliding actors are rigid bodies. 仅当碰撞体是刚体时才生效
		@see PxContactModifyCallback
		*/
		eMODIFY_CONTACTS					= (1<<1),

		/**
		\brief Call contact report callback or trigger callback when this collision pair starts to be in contact.
		If one of the two collision objects is a trigger shape (see #PxShapeFlag::eTRIGGER_SHAPE) 
		then the trigger callback will get called as soon as the other object enters the trigger volume. 
		If none of the two collision objects is a trigger shape then the contact report callback will get 
		called when the actors of this collision pair start to be in contact.
        当碰撞对开始接触时，调用report回调或trigger回调。  
        如果其中一个是触发器形状（即设置了PxShapeFlag::eTRIGGER_SHAPE），那么一旦另一个对象进入触发体积，将调用trigger回调。
        如果两个碰撞对象都不是触发器形状，则当开始接触时调用report回调。
		\note Only takes effect if the colliding actors are rigid bodies. 仅当碰撞体是刚体时才生效
		\note Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised. 只有设置了 eDETECT_DISCRETE_CONTACT 或 eDETECT_CCD_CONTACT 才生效
		@see PxSimulationEventCallback.onContact() PxSimulationEventCallback.onTrigger()
		*/
		eNOTIFY_TOUCH_FOUND					= (1<<2),       // 开始接触时

		/**
		\brief Call contact report callback while this collision pair is in contact
		If none of the two collision objects is a trigger shape then the contact report callback will get 
		called while the actors of this collision pair are in contact.
        如果两者都不是触发器，接触时会调用report回调。
		\note Triggers do not support this event. Persistent trigger contacts need to be tracked separately by observing eNOTIFY_TOUCH_FOUND/eNOTIFY_TOUCH_LOST events.
        触发器不支持此事件。触发器需要通过eNOTIFY_TOUCH_FOUND/eNOTIFY_TOUCH_LOST事件观察。
		\note Only takes effect if the colliding actors are rigid bodies. 仅当碰撞体是刚体时才生效
		\note No report will get sent if the objects in contact are sleeping. 如果接触的对象处于休眠状态，则不会发送report。
		\note Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised. 
              只有设置了 eDETECT_DISCRETE_CONTACT 或 eDETECT_CCD_CONTACT 才生效
		\note If this flag gets enabled while a pair is in touch already, there will be no eNOTIFY_TOUCH_PERSISTS events until the pair loses and regains touch.
              如果此标志在碰撞对已经接触时启用，则不会有 eNOTIFY_TOUCH_PERSISTS 事件，直到它们分开并重新接触。
		@see PxSimulationEventCallback.onContact() PxSimulationEventCallback.onTrigger()
		*/
		eNOTIFY_TOUCH_PERSISTS				= (1<<3),       // 接触中

		/**
		\brief Call contact report callback or trigger callback when this collision pair stops to be in contact
		If one of the two collision objects is a trigger shape (see #PxShapeFlag::eTRIGGER_SHAPE) 
		then the trigger callback will get called as soon as the other object leaves the trigger volume. 
		If none of the two collision objects is a trigger shape then the contact report callback will get 
		called when the actors of this collision pair stop to be in contact.
		\note Only takes effect if the colliding actors are rigid bodies.
		\note This event will also get triggered if one of the colliding objects gets deleted.
		\note Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
              // 要启用 eDETECT_DISCRETE_CONTACT 或者 eDETECT_CCD_CONTACT
		@see PxSimulationEventCallback.onContact() PxSimulationEventCallback.onTrigger()
		*/
		eNOTIFY_TOUCH_LOST					= (1<<4),       // 分开时

		/**
		\brief Call contact report callback when this collision pair is in contact during CCD passes.
		If CCD with multiple passes is enabled, then a fast moving object might bounce on and off the same
		object multiple times. Hence, the same pair might be in contact multiple times during a simulation step.
		This flag will make sure that all the detected collision during CCD will get reported. For performance
		reasons, the system can not always tell whether the contact pair lost touch in one of the previous CCD 
		passes and thus can also not always tell whether the contact is new or has persisted. eNOTIFY_TOUCH_CCD
		just reports when the two collision objects were detected as being in contact during a CCD pass.
        当此碰撞对在 CCD 期间接触时，调用report回调。如果启用了多通道 CCD，那么快速移动的物体可能会和同一对象多次接触又分开。
        因此，同一对碰撞体可能在模拟步骤中多次接触。该标志将确保所有在 CCD 期间检测到的碰撞都会得到报告。 
        因为性能原因，系统不能一直判断接触对是否在前一次 CCD 中失去接触，因此也不能一直判断是新接触的还是一直存在的接触对。  
        eNOTIFY_TOUCH_CCD 仅在 CCD 期间检测到两个碰撞对接触才触发。
		\note Only takes effect if the colliding actors are rigid bodies.   // 仅当碰撞体是刚体时才生效
		\note Trigger shapes are not supported.                             // 触发器不支持。  
		\note Only takes effect if eDETECT_CCD_CONTACT is raised.           // 只有eDETECT_CCD_CONTACT设置了才生效。  
		@see PxSimulationEventCallback.onContact() PxSimulationEventCallback.onTrigger()
		*/
		eNOTIFY_TOUCH_CCD					= (1<<5),

		/**
		\brief Call contact report callback when the contact force between the actors of this collision pair exceeds one of the actor-defined force thresholds.
               当碰撞对之间的接触力超过参与者定义的力阈值时，调用report回调。
		\note Only takes effect if the colliding actors are rigid bodies.
		\note Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
              // 要启用 eDETECT_DISCRETE_CONTACT 或者 eDETECT_CCD_CONTACT
		@see PxSimulationEventCallback.onContact()
		*/
		eNOTIFY_THRESHOLD_FORCE_FOUND		= (1<<6),           // 接触力阈值开始

		/**
		\brief Call contact report callback when the contact force between the actors of this collision pair continues to exceed one of the actor-defined force thresholds.
		\note Only takes effect if the colliding actors are rigid bodies.
		\note If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
		previous one (unless #eNOTIFY_THRESHOLD_FORCE_FOUND has been set in the previous frame).
		\note Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
              // 要启用 eDETECT_DISCRETE_CONTACT 或者 eDETECT_CCD_CONTACT
		@see PxSimulationEventCallback.onContact()
		*/
		eNOTIFY_THRESHOLD_FORCE_PERSISTS	= (1<<7),           // 接触力阈值持续中

		/**
		\brief Call contact report callback when the contact force between the actors of this collision pair falls below one of the actor-defined force thresholds (includes the case where this collision pair stops being in contact).
		\note Only takes effect if the colliding actors are rigid bodies.
		\note If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
		previous one (unless #eNOTIFY_THRESHOLD_FORCE_FOUND or #eNOTIFY_THRESHOLD_FORCE_PERSISTS has been set in the previous frame).
		\note Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
              // 要启用 eDETECT_DISCRETE_CONTACT 或者 eDETECT_CCD_CONTACT
		@see PxSimulationEventCallback.onContact()
		*/
		eNOTIFY_THRESHOLD_FORCE_LOST		= (1<<8),           // 接触力阈值消失

		/**
		\brief Provide contact points in contact reports for this collision pair.
               在此碰撞对的接触报告中提供接触点
		\note Only takes effect if the colliding actors are rigid bodies and if used in combination with the flags eNOTIFY_TOUCH_... or eNOTIFY_THRESHOLD_FORCE_...
		\note Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
              // 要启用 eDETECT_DISCRETE_CONTACT 或者 eDETECT_CCD_CONTACT
		@see PxSimulationEventCallback.onContact() PxContactPair PxContactPair.extractContacts()
		*/
		eNOTIFY_CONTACT_POINTS				= (1<<9),

		/**
		\brief This flag is used to indicate whether this pair generates discrete collision detection contacts. 
               此标志用于指示碰撞对是否生成离散碰撞检测触点
		\note Contacts are only responded to if eSOLVE_CONTACT is enabled.  // 要启用 eSOLVE_CONTACT
		*/
		eDETECT_DISCRETE_CONTACT			= (1<<10),
		
		/**
		\brief This flag is used to indicate whether this pair generates CCD contacts. 
               此标志用于指示碰撞对是否生成 CCD 触点
		\note The contacts will only be responded to if eSOLVE_CONTACT is enabled on this pair. // 要启用 eSOLVE_CONTACT
		\note The scene must have PxSceneFlag::eENABLE_CCD enabled to use this feature.  // 要启用 PxSceneFlag::eENABLE_CCD
		\note Non-static bodies of the pair should have PxRigidBodyFlag::eENABLE_CCD specified for this feature to work correctly.
		\note This flag is not supported with trigger shapes. However, CCD trigger events can be emulated using non-trigger shapes 
		and requesting eNOTIFY_TOUCH_FOUND and eNOTIFY_TOUCH_LOST and not raising eSOLVE_CONTACT on the pair.
		@see PxRigidBodyFlag::eENABLE_CCD
		@see PxSceneFlag::eENABLE_CCD
		*/
		eDETECT_CCD_CONTACT					= (1<<11),

		/**
		\brief Provide pre solver velocities in contact reports for this collision pair.
		If the collision pair has contact reports enabled, the velocities of the rigid bodies before contacts have been solved
		will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
        在碰撞对的接触报告中提供预求解器速度。
        如果碰撞对启用了接触报告，则在report回调中将提供解决接触之前刚体的速度，除非碰撞对失去接触，在这种情况下将不提供数据。
		\note Usually it is not necessary to request these velocities as they will be available by querying the velocity from the provided
		PxRigidActor object directly. However, it might be the case that the velocity of a rigid body gets set while the simulation is running
		in which case the PxRigidActor would return this new velocity in the contact report callback and not the velocity the simulation used.
        通常不需要请求这些速度，因为它们可以通过直接从提供的 PxRigidActor 对象查询速度来获得。 
        然而，在模拟运行时刚体的速度可能会被设置，在这种情况下，PxRigidActor 将在report回调中返回这个新速度，而不是模拟使用的速度。
		@see PxSimulationEventCallback.onContact(), PxContactPairVelocity, PxContactPairHeader.extraDataStream
		*/
		ePRE_SOLVER_VELOCITY				= (1<<12),
		
		/**
		\brief Provide post solver velocities in contact reports for this collision pair.
		If the collision pair has contact reports enabled, the velocities of the rigid bodies after contacts have been solved
		will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
        在此碰撞对的接触报告中提供后求解器速度。
        如果碰撞对启用了接触报告，则接触解决后刚体的速度将在接触报告回调中提供，除非碰撞对失去接触，在这种情况下将不提供数据
		@see PxSimulationEventCallback.onContact(), PxContactPairVelocity, PxContactPairHeader.extraDataStream
		*/
		ePOST_SOLVER_VELOCITY				= (1<<13),
		
		/**
		\brief Provide rigid body poses in contact reports for this collision pair.
		If the collision pair has contact reports enabled, the rigid body poses at the contact event will be provided 
		in the contact report callback unless the pair lost touch in which case no data will be provided.
        在此碰撞对的接触报告中提供刚体姿势。
        如果碰撞对启用了接触报告，则接触事件中的刚体姿势将在接触报告回调中提供，除非碰撞对失去接触，在这种情况下将不提供数据。
		\note Usually it is not necessary to request these poses as they will be available by querying the pose from the provided
		PxRigidActor object directly. However, it might be the case that the pose of a rigid body gets set while the simulation is running
		in which case the PxRigidActor would return this new pose in the contact report callback and not the pose the simulation used.
		Another use case is related to CCD with multiple passes enabled, A fast moving object might bounce on and off the same 
		object multiple times. This flag can be used to request the rigid body poses at the time of impact for each such collision event.
        通常不需要请求这些姿势，因为它们可以通过直接从提供的 PxRigidActor 对象查询姿势来获得。 
        然而，在模拟运行时刚体的姿势可能会被设置，在这种情况下，PxRigidActor 将在联系报告回调中返回这个新姿势，而不是模拟使用的姿势。
        另一个用例与启用多次通过的 CCD 相关，快速移动的物体可能会多次在同一个物体上反弹。 
        此标志可用于请求每个此类碰撞事件发生碰撞时的刚体姿势
		@see PxSimulationEventCallback.onContact(), PxContactPairPose, PxContactPairHeader.extraDataStream
		*/
		eCONTACT_EVENT_POSE					= (1<<14),

		eNEXT_FREE							= (1<<15),        //!< For internal use only.   仅限内部使用

		/**
		\brief Provided default flag to do simple contact processing for this collision pair.
               提供默认标志来为这个碰撞对做简单的接触处理。
		*/
		eCONTACT_DEFAULT					= eSOLVE_CONTACT | eDETECT_DISCRETE_CONTACT,

		/**
		\brief Provided default flag to get commonly used trigger behavior for this collision pair.
               提供默认标志以获取此碰撞对的常用触发行为。
		*/
		eTRIGGER_DEFAULT					= eNOTIFY_TOUCH_FOUND | eNOTIFY_TOUCH_LOST | eDETECT_DISCRETE_CONTACT
	};
};

/**
\brief Bitfield that contains a set of raised flags defined in PxPairFlag.
@see PxPairFlag
*/
typedef PxFlags<PxPairFlag::Enum, PxU16> PxPairFlags;
PX_FLAGS_OPERATORS(PxPairFlag::Enum, PxU16)



/**
\brief Collection of flags describing the filter actions to take for a collision pair.
@see PxFilterFlags PxSimulationFilterShader PxSimulationFilterCallback
*/
struct PxFilterFlag             // 描述碰撞对要采取的过滤器操作的标志集合
{
	enum Enum
	{
		/**
		\brief Ignore the collision pair as long as the bounding volumes of the pair objects overlap.
		Killed pairs will be ignored by the simulation and won't run through the filter again until one
		of the following occurs:
        忽略边界体积重叠的碰撞对。 被杀死的碰撞对将忽略模拟，并且不会再次通过过滤器运行，直到发生以下情况之一：
		\li The bounding volumes of the two objects overlap again (after being separated)
            两个物体的边界体积再次重叠(在分离后)
		\li The user enforces a re-filtering (see #PxScene::resetFiltering())
            用户强制重新过滤(参见#PxScene::resetFiltering())
		@see PxScene::resetFiltering()
		*/
		eKILL				= (1<<0),

		/**
		\brief Ignore the collision pair as long as the bounding volumes of the pair objects overlap or until filtering relevant data changes for one of the collision objects.
		Suppressed pairs will be ignored by the simulation and won't make another filter request until one
		of the following occurs:
        只要碰撞对的边界体积重叠，或者直到碰撞对象之一的过滤相关数据发生变化，就忽略碰撞对。
        模拟将忽略被抑制的对，并且不会发出另一个过滤请求，直到发生以下情况之一：
		\li Same conditions as for killed pairs (see #eKILL)
            与被杀死对相同的条件（参见#eKILL）
		\li The filter data or the filter object attributes change for one of the collision objects
            碰撞对象之一的过滤器数据或过滤器对象属性发生变化
		@see PxFilterData PxFilterObjectAttributes
		*/
		eSUPPRESS			= (1<<1),

		/**
		\brief Invoke the filter callback (#PxSimulationFilterCallback::pairFound()) for this collision pair.
               调用此碰撞对的过滤器回调
		@see PxSimulationFilterCallback
		*/
		eCALLBACK			= (1<<2),

		/**
		\brief Track this collision pair with the filter callback mechanism.
		When the bounding volumes of the collision pair lose contact, the filter callback #PxSimulationFilterCallback::pairLost()
		will be invoked. Furthermore, the filter status of the collision pair can be adjusted through #PxSimulationFilterCallback::statusChange()
		once per frame (until a pairLost() notification occurs).
        使用过滤器回调机制跟踪此碰撞对。  
        当碰撞对的边界体积失去接触时，调用过滤器回调。
        此外，可以通过#PxSimulationFilterCallback::statusChange()每帧调整一次碰撞对的过滤器状态(直到pairLost())。
		@see PxSimulationFilterCallback
		*/
		eNOTIFY				= (1<<3) | eCALLBACK,

		/**
		\brief Provided default to get standard behavior:
		The application configure the pair's collision properties once when bounding volume overlap is found and
		doesn't get asked again about that pair until overlap status or filter properties changes, or re-filtering is requested.
		No notification is provided when bounding volume overlap is lost
		The pair will not be killed or suppressed, so collision detection will be processed
        提供默认以获得标准行为：
        当发现边界体积重叠时，应用程序配置对的碰撞属性一次，并且在重叠状态或过滤器属性更改或请求重新过滤之前，不会再次询问该对。
        边界体积重叠丢失时不提供通知，碰撞对不会被杀死或抑制，因此将处理碰撞检测
		*/
		eDEFAULT = 0
	};
};

/**
\brief Bitfield that contains a set of raised flags defined in PxFilterFlag.
@see PxFilterFlag
*/
typedef PxFlags<PxFilterFlag::Enum, PxU16> PxFilterFlags;
PX_FLAGS_OPERATORS(PxFilterFlag::Enum, PxU16)


/**
\brief PxFilterData is user-definable data which gets passed into the collision filtering shader and/or callback.
PxFilterData 是用户可定义的数据，它被传递到碰撞过滤着色器和/或回调中。
@see PxShape.setSimulationFilterData() PxShape.getSimulationFilterData()  PxSimulationFilterShader PxSimulationFilterCallback
*/
struct PxFilterData
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	PX_INLINE PxFilterData(const PxEMPTY)
	{
	}

	/**
	\brief Default constructor.
	*/
	PX_INLINE PxFilterData() 
	{
		word0 = word1 = word2 = word3 = 0;
	}

	/**
	\brief Copy constructor.
	*/
	PX_INLINE PxFilterData(const PxFilterData& fd) : word0(fd.word0), word1(fd.word1), word2(fd.word2), word3(fd.word3)	{}

	/**
	\brief Constructor to set filter data initially.
	*/
	PX_INLINE PxFilterData(PxU32 w0, PxU32 w1, PxU32 w2, PxU32 w3) : word0(w0), word1(w1), word2(w2), word3(w3) {}

	/**
	\brief (re)sets the structure to the default.	
	*/
	PX_INLINE void setToDefault()
	{
		*this = PxFilterData();
	}

	/**
	\brief Assignment operator
	*/
	PX_INLINE void operator = (const PxFilterData& fd)
	{
		word0 = fd.word0;
		word1 = fd.word1;
		word2 = fd.word2;
		word3 = fd.word3;
	}

	/**
	\brief Comparison operator to allow use in Array.
	*/
	PX_INLINE bool operator == (const PxFilterData& a) const
	{
		return a.word0 == word0 && a.word1 == word1 && a.word2 == word2 && a.word3 == word3;
	}

	/**
	\brief Comparison operator to allow use in Array.
	*/
	PX_INLINE bool operator != (const PxFilterData& a) const
	{
		return !(a == *this);
	}

	PxU32 word0;
	PxU32 word1;
	PxU32 word2;
	PxU32 word3;
};


/**
\brief Identifies each type of filter object.
@see PxGetFilterObjectType()
*/
struct PxFilterObjectType       // 标识每种类型的过滤器对象
{
	enum Enum
	{
		/**
		\brief A static rigid body          静态刚体
		@see PxRigidStatic
		*/
		eRIGID_STATIC,

		/**
		\brief A dynamic rigid body         动态刚体
		@see PxRigidDynamic
		*/
		eRIGID_DYNAMIC,

		/**
		\brief An articulation              关节
		@see PxArticulation
		*/
		eARTICULATION,

		//brief internal use only!
		eMAX_TYPE_COUNT = 16,

		//brief internal use only!
		eUNDEFINED = eMAX_TYPE_COUNT-1
	};
};


// For internal use only
struct PxFilterObjectFlag
{
	enum Enum
	{
		eKINEMATIC		= (1<<4),
		eTRIGGER		= (1<<5)
	};
};


/**
\brief Structure which gets passed into the collision filtering shader and/or callback providing additional information on objects of a collision pair
传递到碰撞过滤着色器 and/or 回调的结构，提供有关碰撞对的对象的附加信息  
@see PxSimulationFilterShader PxSimulationFilterCallback getActorType() PxFilterObjectIsKinematic() PxFilterObjectIsTrigger()
*/
typedef PxU32 PxFilterObjectAttributes;


/**
\brief Extract filter object type from the filter attributes of a collision pair object
\param[in] attr The filter attribute of a collision pair object
\return The type of the collision pair object.
@see PxFilterObjectType
*/
PX_INLINE PxFilterObjectType::Enum PxGetFilterObjectType(PxFilterObjectAttributes attr)
{
	return PxFilterObjectType::Enum(attr & (PxFilterObjectType::eMAX_TYPE_COUNT-1));
}


/**
\brief Specifies whether the collision object belongs to a kinematic rigid body
\param[in] attr The filter attribute of a collision pair object
\return True if the object belongs to a kinematic rigid body, else false
@see PxRigidBodyFlag::eKINEMATIC
*/
PX_INLINE bool PxFilterObjectIsKinematic(PxFilterObjectAttributes attr)
{
	return (attr & PxFilterObjectFlag::eKINEMATIC) != 0;
}


/**
\brief Specifies whether the collision object is a trigger shape
\param[in] attr The filter attribute of a collision pair object
\return True if the object is a trigger shape, else false
@see PxShapeFlag::eTRIGGER_SHAPE
*/
PX_INLINE bool PxFilterObjectIsTrigger(PxFilterObjectAttributes attr)
{
	return (attr & PxFilterObjectFlag::eTRIGGER) != 0;
}


/**
\brief Filter shader to specify handling of collision pairs.

Collision filtering is a mechanism to specify how a pair of potentially colliding objects should be processed by the
simulation. A pair of objects is potentially colliding if the bounding volumes of the two objects overlap.
In short, a collision filter decides whether a collision pair should get processed, temporarily ignored or discarded.
If a collision pair should get processed, the filter can additionally specify how it should get processed, for instance,
whether contacts should get resolved, which callbacks should get invoked or which reports should be sent etc.

\note A default implementation of a filter shader is provided in the PhysX extensions library, see #PxDefaultSimulationFilterShader.

@see PxSceneDesc.filterShader PxSimulationFilterCallback
*/

/**
\brief Filter method to specify how a pair of potentially colliding objects should be processed.

Return the PxFilterFlag flags and set the PxPairFlag flags to define what the simulation should do with the given collision pair.

This methods gets called when:
\li The bounding volumes of two objects start to overlap.
\li The bounding volumes of two objects overlap and the filter data or filter attributes of one of the objects changed
\li A re-filtering was forced through resetFiltering() (see #PxScene::resetFiltering())
\li Filtering is requested in scene queries

\note Certain pairs of objects are always ignored and this method does not get called. This is the case for the
following pairs:

\li Pair of static rigid actors
\li A static rigid actor and a kinematic actor (unless one is a trigger or if explicitly enabled through PxPairFilteringMode::eKEEP)
\li Two kinematic actors (unless one is a trigger or if explicitly enabled through PxPairFilteringMode::eKEEP)
\li Two jointed rigid bodies and the joint was defined to disable collision
\li Two articulation links if connected through an articulation joint

\note This is a performance critical method and should be stateless. You should neither access external objects 
from within this method nor should you call external methods that are not inlined. If you need a more complex
logic to filter a collision pair then use the filter callback mechanism for this pair (see #PxSimulationFilterCallback,
#PxFilterFlag::eCALLBACK, #PxFilterFlag::eNOTIFY).

\param[in] attributes0 The filter attribute of the first object
\param[in] filterData0 The custom filter data of the first object
\param[in] attributes1 The filter attribute of the second object
\param[in] filterData1 The custom filter data of the second object
\param[out] pairFlags Flags giving additional information on how an accepted pair should get processed
\param[in] constantBlock The constant global filter data (see #PxSceneDesc.filterShaderData)
\param[in] constantBlockSize Size of the global filter data (see #PxSceneDesc.filterShaderDataSize)
\return Filter flags defining whether the pair should be discarded, temporarily ignored, processed and whether the
filter callback should get invoked for this pair.

@see PxSimulationFilterCallback PxFilterData PxFilterObjectAttributes PxFilterFlag PxFilterFlags PxPairFlag PxPairFlags
*/

typedef PxFilterFlags (*PxSimulationFilterShader)
	(PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
	 PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	 PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize);



/**
\brief Filter callback to specify handling of collision pairs.
过滤回调以指定碰撞对的处理。 

This class is provided to implement more complex and flexible collision pair filtering logic, for instance, taking
the state of the user application into account. Filter callbacks also give the user the opportunity to track collision
pairs and update their filter state.
提供这个类是为了实现更复杂和灵活的碰撞对过滤逻辑，例如，考虑用户应用程序的状态。 
过滤器回调还使用户有机会跟踪碰撞对并更新其过滤器状态。

You might want to check the documentation on #PxSimulationFilterShader as well since it includes more general information
on filtering.
您可能还想查看有关 #PxSimulationFilterShader 的文档，因为它包含有关过滤的更多一般信息。

\note SDK state should not be modified from within the callbacks. In particular objects should not
be created or destroyed. If state modification is needed then the changes should be stored to a buffer
and performed after the simulation step.
SDK 状态不应在回调中修改，特别是不应创建或销毁对象。如果需要修改状态，则应将更改存储到缓冲区并在模拟步骤之后执行。

\note The callbacks may execute in user threads or simulation threads, possibly simultaneously. The corresponding objects 
may have been deleted by the application earlier in the frame. It is the application's responsibility to prevent race conditions
arising from using the SDK API in the callback while an application thread is making write calls to the scene, and to ensure that
the callbacks are thread-safe. Return values which depend on when the callback is called during the frame will introduce nondeterminism 
into the simulation.
回调可以在用户线程或模拟线程中执行，可能同时执行。
相应的对象可能已在框架中较早的时候被应用程序删除。
应用程序有责任防止在应用程序线程对场景进行写入调用时在回调中使用 SDK API 产生竞争条件，并确保回调是线程安全的。
取决于在帧期间调用回调的时间的返回值将在模拟中引入不确定性。

@see PxSceneDesc.filterCallback PxSimulationFilterShader
*/
class PxSimulationFilterCallback
{
public:

	/**
	\brief Filter method to specify how a pair of potentially colliding objects should be processed.
           Filter 方法来指定如何处理一对潜在的碰撞对象

	This method gets called when the filter flags returned by the filter shader (see #PxSimulationFilterShader)
	indicate that the filter callback should be invoked (#PxFilterFlag::eCALLBACK or #PxFilterFlag::eNOTIFY set).
	Return the PxFilterFlag flags and set the PxPairFlag flags to define what the simulation should do with the given 
	collision pair.
    当过滤器着色器返回的过滤器标志（请参阅#PxSimulationFilterShader）指示应调用
    过滤器回调（#PxFilterFlag::eCALLBACK 或 #PxFilterFlag::eNOTIFY 设置）时，将调用此方法。
    返回 PxFilterFlag 标志并设置 PxPairFlag 标志以定义模拟应对给定碰撞对执行的操作。

	\param[in] pairID Unique ID of the collision pair used to issue filter status changes for the pair (see #statusChange())
	\param[in] attributes0 The filter attribute of the first object
	\param[in] filterData0 The custom filter data of the first object
	\param[in] a0 Actor pointer of the first object
	\param[in] s0 Shape pointer of the first object (NULL if the object has no shapes)
	\param[in] attributes1 The filter attribute of the second object
	\param[in] filterData1 The custom filter data of the second object
	\param[in] a1 Actor pointer of the second object
	\param[in] s1 Shape pointer of the second object (NULL if the object has no shapes)
	\param[in,out] pairFlags In: Pair flags returned by the filter shader. Out: Additional information on how an accepted pair should get processed
	\return Filter flags defining whether the pair should be discarded, temporarily ignored or processed and whether the pair
	should be tracked and send a report on pair deletion through the filter callback

	@see PxSimulationFilterShader PxFilterData PxFilterObjectAttributes PxFilterFlag PxPairFlag
	*/
	virtual		PxFilterFlags	pairFound(	PxU32 pairID,
		PxFilterObjectAttributes attributes0, PxFilterData filterData0, const PxActor* a0, const PxShape* s0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1, const PxActor* a1, const PxShape* s1,
		PxPairFlags& pairFlags) = 0;

	/**
	\brief Callback to inform that a tracked collision pair is gone.
           回调以通知跟踪的碰撞对已消失。

	This method gets called when a collision pair disappears or gets re-filtered. Only applies to
	collision pairs which have been marked as filter callback pairs (#PxFilterFlag::eNOTIFY set in #pairFound()).
    当碰撞对消失或被重新过滤时，将调用此方法。 仅适用于已标记为过滤器回调对的碰撞对（#pairFound() 中设置的#PxFilterFlag::eNOTIFY）。

	\param[in] pairID Unique ID of the collision pair that disappeared
	\param[in] attributes0 The filter attribute of the first object
	\param[in] filterData0 The custom filter data of the first object
	\param[in] attributes1 The filter attribute of the second object
	\param[in] filterData1 The custom filter data of the second object
	\param[in] objectRemoved True if the pair was lost because one of the objects got removed from the scene

	@see pairFound() PxSimulationFilterShader PxFilterData PxFilterObjectAttributes
	*/
	virtual		void			pairLost(	PxU32 pairID,
		PxFilterObjectAttributes attributes0,
		PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,
		PxFilterData filterData1,
		bool objectRemoved) = 0;

	/**
	\brief Callback to give the opportunity to change the filter state of a tracked collision pair.
           回调以提供机会更改跟踪碰撞对的过滤器状态。

	This method gets called once per simulation step to let the application change the filter and pair
	flags of a collision pair that has been reported in #pairFound() and requested callbacks by
	setting #PxFilterFlag::eNOTIFY. To request a change of filter status, the target pair has to be
	specified by its ID, the new filter and pair flags have to be provided and the method should return true.
    每个模拟步骤调用此方法一次，让应用程序更改已在 #pairFound() 中报告的碰撞对的过滤器和对标志，并通过设置 #PxFilterFlag::eNOTIFY 请求回调。 
    要请求更改过滤器状态，必须通过其 ID 指定目标对，必须提供新的过滤器和对标志，并且该方法应返回 true

	\note If this method changes the filter status of a collision pair and the pair should keep being tracked
	by the filter callbacks then #PxFilterFlag::eNOTIFY has to be set.
          如果此方法更改了碰撞对的过滤器状态，并且过滤器回调应继续跟踪该对，则必须设置 #PxFilterFlag::eNOTIFY。

	\note The application is responsible to ensure that this method does not get called for pairs that have been
	reported as lost, see #pairLost().
          应用程序负责确保不会为已报告为丢失的对调用此方法，请参阅#pairLost()。

	\param[out] pairID ID of the collision pair for which the filter status should be changed
	\param[out] pairFlags The new pairFlags to apply to the collision pair
	\param[out] filterFlags The new filterFlags to apply to the collision pair
	\return True if the changes should be applied. In this case the method will get called again. False if
	no more status changes should be done in the current simulation step. In that case the provided flags will be discarded.

	@see pairFound() pairLost() PxFilterFlag PxPairFlag
	*/
	virtual		bool			statusChange(PxU32& pairID, PxPairFlags& pairFlags, PxFilterFlags& filterFlags) = 0;

protected:
	virtual						~PxSimulationFilterCallback() {}
};

struct PxPairFilteringMode
{
	enum Enum
	{
		/**
		Output pair from BP, potentially send to user callbacks, create regular interaction object.
		Enable contact pair filtering between kinematic/static or kinematic/kinematic rigid bodies.
		By default contacts between these are suppressed (see #PxFilterFlag::eSUPPRESS) and don't get reported to the filter mechanism.
		Use this mode if these pairs should go through the filtering pipeline nonetheless.
        来自 BP 的输出对，可能发送给用户回调，创建常规交互对象。
        在运动学/静态或运动学/运动学刚体之间启用接触对过滤。
        默认情况下，它们之间的联系被抑制（参见#PxFilterFlag::eSUPPRESS）并且不会被报告给过滤机制。
        如果这些对仍然应该通过过滤管道，请使用此模式。
		\note This mode is not mutable, and must be set in PxSceneDesc at scene creation.
              此模式不可变，必须在场景创建时在 PxSceneDesc 中设置。
		*/
		eKEEP,

		/**
		Output pair from BP, create interaction marker. Can be later switched to regular interaction.
        来自 BP 的输出对，创建交互标记。 以后可以切换到常规交互
		*/
		eSUPPRESS,

		/**
		Don't output pair from BP. Cannot be later switched to regular interaction, needs "resetFiltering" call.
        不要从 BP 输出对。 以后不能切换到常规交互，需要“resetFiltering”调用。
		*/
		eKILL,

		/**
		Default is eSUPPRESS for compatibility with previous PhysX versions.
        默认为 eSUPPRESS 以兼容以前的 PhysX 版本。
		*/
		eDEFAULT = eSUPPRESS
	};
};


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
