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

#ifndef PX_PHYSICS_NX_SHAPE
#define PX_PHYSICS_NX_SHAPE
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "common/PxBase.h"
#include "geometry/PxGeometry.h"
#include "geometry/PxGeometryHelpers.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxBoxGeometry;
class PxSphereGeometry;
class PxCapsuleGeometry;
class PxPlaneGeometry;
class PxConvexMeshGeometry;
class PxTriangleMeshGeometry;
class PxHeightFieldGeometry;
class PxRigidActor;
struct PxFilterData;
struct PxRaycastHit;
struct PxSweepHit;

/**
\brief Flags which affect the behavior of PxShapes. // 影响 PxShapes 行为的标志。
@see PxShape PxShape.setFlag()
*/
struct PxShapeFlag
{
	enum Enum
	{
		/**
		\brief The shape will partake in collision in the physical simulation.
               该形状将参与物理模拟中的碰撞。
		\note It is illegal to raise the eSIMULATION_SHAPE and eTRIGGER_SHAPE flags.
		      In the event that one of these flags is already raised the sdk will reject any 
		      attempt to raise the other.  To raise the eSIMULATION_SHAPE first ensure that 
		      eTRIGGER_SHAPE is already lowered.
              提高 eSIMULATION_SHAPE 和 eTRIGGER_SHAPE 标志是非法的。
              如果这些标志中的一个已经被引发，sdk 将拒绝任何引发另一个的尝试。 
              要提高 eSIMULATION_SHAPE 首先确保 eTRIGGER_SHAPE 已经降低
		\note This flag has no effect if simulation is disabled for the corresponding actor (see #PxActorFlag::eDISABLE_SIMULATION).
              如果为相应的actor 禁用模拟（请参阅#PxActorFlag::eDISABLE_SIMULATION），则此标志无效。
		@see PxSimulationEventCallback.onContact() PxScene.setSimulationEventCallback() PxShape.setFlag(), PxShape.setFlags()
		*/
		eSIMULATION_SHAPE				= (1<<0),

		/**
		\brief The shape will partake in scene queries (ray casts, overlap tests, sweeps, ...).
               该形状将参与场景查询（光线投射、重叠测试、扫描等）。
		*/
		eSCENE_QUERY_SHAPE				= (1<<1),

		/**
		\brief The shape is a trigger which can send reports whenever other shapes enter/leave its volume.
               该形状是一个触发器，可以在其他形状进入/离开其体积时发送报告。
		\note Triangle meshes and heightfields can not be triggers. Shape creation will fail in these cases.
              三角形网格和高度场不能被触发。 在这些情况下，形状创建将失败。
		\note Shapes marked as triggers do not collide with other objects. If an object should act both
		      as a trigger shape and a collision shape then create a rigid body with two shapes, one being a 
		      trigger shape and the other a collision shape. 	It is illegal to raise the eTRIGGER_SHAPE and 
		      eSIMULATION_SHAPE flags on a single PxShape instance.  In the event that one of these flags is already 
		      raised the sdk will reject any attempt to raise the other.  To raise the eTRIGGER_SHAPE flag first 
		      ensure that eSIMULATION_SHAPE flag is already lowered.
              标记为触发器的形状不会与其他对象发生碰撞。 
              如果一个对象应该同时作为触发器形状和碰撞形状，那么创建一个具有两种形状的刚体，一个是触发器形状，另一个是碰撞形状。 
              在单个 PxShape 实例上引发 eTRIGGER_SHAPE 和 eSIMULATION_SHAPE 标志是非法的。 
              如果这些标志中的一个已经被引发，sdk 将拒绝任何引发另一个的尝试。 
              要提升 eTRIGGER_SHAPE 标志，首先确保 eSIMULATION_SHAPE 标志已经降低。
		\note Trigger shapes will no longer send notification events for interactions with other trigger shapes.
              触发器形状将不再发送与其他触发器形状交互的通知事件。
		\note Shapes marked as triggers are allowed to participate in scene queries, provided the eSCENE_QUERY_SHAPE flag is set. 
              如果设置了 eSCENE_QUERY_SHAPE 标志，则允许标记为触发器的形状参与场景查询。
		\note This flag has no effect if simulation is disabled for the corresponding actor (see #PxActorFlag::eDISABLE_SIMULATION).
              如果为相应的actor 禁用模拟（请参阅#PxActorFlag::eDISABLE_SIMULATION），则此标志无效。
		@see PxSimulationEventCallback.onTrigger() PxScene.setSimulationEventCallback() PxShape.setFlag(), PxShape.setFlags()
		*/
		eTRIGGER_SHAPE					= (1<<2),

		/**
		\brief Enable debug renderer for this shape     为此形状启用调试渲染器
		@see PxScene.getRenderBuffer() PxRenderBuffer PxVisualizationParameter
		*/
		eVISUALIZATION					= (1<<3)
	};
};

/**
\brief collection of set bits defined in PxShapeFlag.
@see PxShapeFlag
*/
typedef PxFlags<PxShapeFlag::Enum,PxU8> PxShapeFlags;
PX_FLAGS_OPERATORS(PxShapeFlag::Enum,PxU8)


/**
\brief Abstract class for collision shapes.     碰撞形状的抽象类。
Shapes are shared, reference counted objects.   形状是共享的、引用计数的对象。
An instance can be created by calling the createShape() method of the PxRigidActor class, or the createShape() method of the PxPhysics class.
可以通过调用 PxRigidActor 类的 createShape() 方法或 PxPhysics 类的 createShape() 方法来创建实例。

<h3>Visualizations</h3>
\li PxVisualizationParameter::eCOLLISION_AABBS
\li PxVisualizationParameter::eCOLLISION_SHAPES
\li PxVisualizationParameter::eCOLLISION_AXES

@see PxPhysics.createShape() PxRigidActor.createShape() PxBoxGeometry PxSphereGeometry PxCapsuleGeometry PxPlaneGeometry PxConvexMeshGeometry
PxTriangleMeshGeometry PxHeightFieldGeometry
*/
class PxShape : public PxBase
{
public:

	/**
	\brief Decrements the reference count of a shape and releases it if the new reference count is zero.
           如果新的引用计数为零，则递减形状的引用计数并释放它。
	Note that in releases prior to PhysX 3.3 this method did not have reference counting semantics and was used to destroy a shape 
	created with PxActor::createShape(). In PhysX 3.3 and above, this usage is deprecated, instead, use PxRigidActor::detachShape() to detach
	a shape from an actor. If the shape to be detached was created with PxActor::createShape(), the actor holds the only counted reference,
	and so when the shape is detached it will also be destroyed. 
    请注意，在 PhysX 3.3 之前的版本中，此方法没有引用计数语义，并且用于销毁使用 PxActor::createShape() 创建的形状。 
    在 PhysX 3.3 及更高版本中，不推荐使用此用法，而是使用 PxRigidActor::detachShape() 将形状与 actor 分离。 
    如果要分离的形状是使用 PxActor::createShape() 创建的，actor 持有唯一计数的引用，因此当形状分离时，它也会被销毁。
	@see PxRigidActor::createShape() PxPhysics::createShape() PxRigidActor::attachShape() PxRigidActor::detachShape()
	*/
	virtual		void					release() = 0;

	/**
	\brief Returns the reference count of the shape.    返回形状的引用计数。
	At creation, the reference count of the shape is 1. Every actor referencing this shape increments the
	count by 1.	When the reference count reaches 0, and only then, the shape gets destroyed automatically.
    在创建时，形状的引用计数为 1。每个引用此形状的 actor 将计数增加 1。当引用计数达到 0 时，形状才会自动销毁。
	\return the current reference count.
	*/
	virtual		PxU32					getReferenceCount() const = 0;

	/**
	\brief Acquires a counted reference to a shape. 获取对形状的计数引用。
	This method increases the reference count of the shape by 1. Decrement the reference count by calling release()
    此方法将形状的引用计数增加 1。通过调用 release() 减少引用计数
	*/
	virtual		void					acquireReference() = 0;

	/**
	\brief Get the geometry type of the shape.      获取形状的几何类型。
	\return Type of shape geometry.
	@see PxGeometryType
	*/
	virtual		PxGeometryType::Enum	getGeometryType() const = 0;

	/**
	\brief Adjust the geometry of the shape.        调整几何形状数据
	\note The type of the passed in geometry must match the geometry type of the shape.
          传入的几何类型必须与形状的几何类型匹配。
	\note It is not allowed to change the geometry type of a shape.
          不允许更改形状的几何类型。
	\note This function does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
          当对象位于旧几何或新几何之上时，此函数不保证正确/连续的行为。
	\param[in] geometry New geometry of the shape.
	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		void					setGeometry(const PxGeometry& geometry) = 0;


	/**
	\brief Retrieve the geometry from the shape in a PxGeometryHolder wrapper class.
           从 PxGeometryHolder 包装类中的形状中检索几何体。
	\return a PxGeometryHolder object containing the geometry;
	@see PxGeometry PxGeometryType getGeometryType() setGeometry()
	*/
	virtual		PxGeometryHolder		getGeometry() const = 0;


	/**
	\brief Fetch the geometry of the shape.     获取形状的几何形状。
	\note If the type of geometry to extract does not match the geometry type of the shape
	      then the method will return false and the passed in geometry descriptor is not modified.
          如果要提取的几何类型与形状的几何类型不匹配，则该方法将返回 false 并且不会修改传入的几何描述符。
	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false
	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getBoxGeometry(PxBoxGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.     获取形状的几何形状。
	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.
	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false
	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getSphereGeometry(PxSphereGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.
	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.
	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false
	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getCapsuleGeometry(PxCapsuleGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.
	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.
	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false
	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getPlaneGeometry(PxPlaneGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.
	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.
	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false
	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getConvexMeshGeometry(PxConvexMeshGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.
	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.
	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false
	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getTriangleMeshGeometry(PxTriangleMeshGeometry& geometry) const = 0;


	/**
	\brief Fetch the geometry of the shape.
	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.
	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false
	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getHeightFieldGeometry(PxHeightFieldGeometry& geometry) const = 0;

	/**
	\brief Retrieves the actor which this shape is associated with.
           检索与此形状关联的演员。
	\return The actor this shape is associated with, if it is an exclusive shape, else NULL
            与此形状相关联的演员，如果它是排他的形状，否则为 NULL
	@see PxRigidStatic, PxRigidDynamic, PxArticulationLink
	*/
	virtual		PxRigidActor*			getActor() const = 0;


/************************************************************************************************/

/** @name Pose Manipulation     姿势操纵
*/
//@{

	/**
	\brief Sets the pose of the shape in actor space, i.e. relative to the actors to which they are attached.
           在actor空间中设置形状的姿势，即相对于它们所附加的actor。
	
	This transformation is identity by default.     默认情况下，此转换是标识。
	The local pose is an attribute of the shape, and so will apply to all actors to which the shape is attached.
    局部姿势是形状的一个属性，因此将应用于形状附加到的所有演员。

	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.
    Sleeping：不会自动唤醒关联的actor。

	<i>Note:</i> Does not automatically update the inertia properties of the owning actor (if applicable); use the
	PhysX extensions method #PxRigidBodyExt::updateMassAndInertia() to do this.
    不自动更新拥有者的惯性属性（如果适用）； 使用 PhysX 扩展方法 #PxRigidBodyExt::updateMassAndInertia() 来做到这一点。

	<b>Default:</b> the identity transform
    默认：身份转换

	\param[in] pose	The new transform from the actor frame to the shape frame. <b>Range:</b> rigid body transform
	@see getLocalPose() 
	*/
	virtual		void					setLocalPose(const PxTransform& pose)		= 0;

	/**
	\brief Retrieves the pose of the shape in actor space, i.e. relative to the actor they are owned by.
	This transformation is identity by default.
           检索演员空间中形状的姿势，即相对于它们所属的演员。
           默认情况下，此转换是标识。
	\return Pose of shape relative to the actor's frame.
	@see setLocalPose() 
	*/
	virtual		PxTransform				getLocalPose()					const	= 0;

//@}
/************************************************************************************************/

/** @name Collision Filtering       碰撞过滤
*/
//@{

	/**
	\brief Sets the user definable collision filter data.   设置用户可定义的碰撞过滤器数据。
	
	<b>Sleeping:</b> Does wake up the actor if the filter data change causes a formerly suppressed
	collision pair to be enabled.
    休眠：如果过滤器数据更改导致启用先前抑制的碰撞对，则唤醒actor。

	<b>Default:</b> (0,0,0,0)
	@see getSimulationFilterData() 
	*/
	virtual		void					setSimulationFilterData(const PxFilterData& data)	= 0;

	/**
	\brief Retrieves the shape's collision filter data.
	@see setSimulationFilterData() 
	*/
	virtual		PxFilterData			getSimulationFilterData()					const	= 0;

	/**
	\brief Sets the user definable query filter data.   设置用户可自定义的查询过滤数据
	<b>Default:</b> (0,0,0,0)
	@see getQueryFilterData() 
	*/
	virtual		void					setQueryFilterData(const PxFilterData& data)	= 0;

	/**
	\brief Retrieves the shape's Query filter data.
	@see setQueryFilterData() 
	*/
	virtual		PxFilterData			getQueryFilterData()					const	= 0;

//@}
/************************************************************************************************/

	/**
	\brief Assigns material(s) to the shape.    为形状指定材质
	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.
    Sleeping：不会自动唤醒关联的actor
	\param[in] materials List of material pointers to assign to the shape. See #PxMaterial
	\param[in] materialCount The number of materials provided.
	@see PxPhysics.createMaterial() getMaterials() 
	*/
	virtual		void					setMaterials(PxMaterial*const* materials, PxU16 materialCount)	= 0;

	/**
	\brief Returns the number of materials assigned to the shape.
           返回分配给形状的材质数量。
	You can use #getMaterials() to retrieve the material pointers.
    可以使用 #getMaterials() 来检索材料指针。
	\return Number of materials associated with this shape. 与此形状关联的材料数量。
	@see PxMaterial getMaterials()
	*/
	virtual		PxU16					getNbMaterials()		const	= 0;

	/**
	\brief Retrieve all the material pointers associated with the shape.
           检索与形状关联的所有材质指针。
	You can retrieve the number of material pointers by calling #getNbMaterials()
	Note: Removing materials with #PxMaterial::release() will invalidate the pointer of the released material.
    您可以通过调用#getNbMaterials() 来检索材料指针的数量
    注意：使用#PxMaterial::release() 删除材料将使释放的材料的指针无效。
	\param[out] userBuffer The buffer to store the material pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first material pointer to be retrieved
	\return Number of material pointers written to the buffer.
	@see PxMaterial getNbMaterials() PxMaterial::release()
	*/
	virtual		PxU32					getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	= 0;
	
	/**
	\brief Retrieve material from given triangle index. 从给定的三角形索引中检索材料。

	The input index is the internal triangle index as used inside the SDK. This is the index
	returned to users by various SDK functions such as raycasts.
    输入索引是 SDK 内部使用的内部三角形索引。 这是光线投射等各种SDK功能返回给用户的索引。
	
	This function is only useful for triangle meshes or heightfields, which have per-triangle
	materials. For other shapes the function returns the single material associated with the
	shape, regardless of the index.
    此功能仅适用于具有每个三角形材料的三角形网格或高度场。 
    对于其他形状，该函数返回与该形状关联的单一材质，而不管索引如何。

	\param[in] faceIndex The internal triangle index whose material you want to retrieve.
	\return Material from input triangle
	\note If faceIndex value of 0xFFFFffff is passed as an input for mesh and heightfield shapes, this function will issue a warning and return NULL.
          如果 faceIndex 值 0xFFFFffff 作为网格和高度场形状的输入传递，则此函数将发出警告并返回 NULL。
	\note Scene queries set the value of PxQueryHit::faceIndex to 0xFFFFffff whenever it is undefined or does not apply.
          场景查询将 PxQueryHit::faceIndex 的值设置为 0xFFFFffff，只要它未定义或不适用。
	@see PxMaterial getNbMaterials() PxMaterial::release()
	*/
	virtual		PxMaterial*				getMaterialFromInternalFaceIndex(PxU32 faceIndex) const = 0;

	/**
	\brief Sets the contact offset.     设置接触偏移

	Shapes whose distance is less than the sum of their contactOffset values will generate contacts. The contact offset must be positive and
	greater than the rest offset. Having a contactOffset greater than than the restOffset allows the collision detection system to
	predictively enforce the contact constraint even when the objects are slightly separated. This prevents jitter that would occur
	if the constraint were enforced only when shapes were within the rest distance.
    距离小于其 contactOffset 值总和的形状将生成接触。 接触偏移必须为正且大于静止偏移。 
    使 contactOffset 大于 restOffset 允许碰撞检测系统预测性地强制执行接触约束，即使对象稍微分开时也是如此。 
    这可以防止仅当形状在静止距离内时才强制执行约束时发生的抖动。

	<b>Default:</b> 0.02f * PxTolerancesScale::length
	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.
    睡眠：不会自动唤醒关联的actor
	\param[in] contactOffset <b>Range:</b> [maximum(0,restOffset), PX_MAX_F32)
	@see getContactOffset PxTolerancesScale setRestOffset
	*/
	virtual		void					setContactOffset(PxReal contactOffset)	= 0;

	/**
	\brief Retrieves the contact offset. 
	\return The contact offset of the shape.
	@see setContactOffset()
	*/
	virtual		PxReal					getContactOffset() const	= 0;

	/**
	\brief Sets the rest offset.        设置静止偏移

	Two shapes will come to rest at a distance equal to the sum of their restOffset values. If the restOffset is 0, they should converge to touching 
	exactly.  Having a restOffset greater than zero is useful to have objects slide smoothly, so that they do not get hung up on irregularities of 
	each others' surfaces.
    两个形状的静止距离等于它们的 restOffset 值之和。 如果 restOffset 为 0，它们应该收敛到精确接触。
    使 restOffset 大于零对于使对象平滑滑动很有用，这样它们就不会挂在彼此表面的不规则处。

	<b>Default:</b> 0.0f
	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.
	\param[in] restOffset	<b>Range:</b> (-PX_MAX_F32, contactOffset)
	@see getRestOffset setContactOffset
	*/
	virtual		void					setRestOffset(PxReal restOffset)	= 0;

	/**
	\brief Retrieves the rest offset. 
	\return The rest offset of the shape.
	@see setRestOffset()
	*/
	virtual		PxReal					getRestOffset() const	= 0;


	/**
	\brief Sets torsional patch radius.     设置扭转补丁半径。
	
	This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
	will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
	so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate 
	rotational friction introduced by the compression of contacting surfaces.
    这定义了用于施加扭转摩擦的接触面的半径。 如果半径为 0，则不会施加扭转摩擦。 
    如果半径 > 0，则会施加一些扭转摩擦。 
    这与穿透深度成正比，因此，如果形状分开或穿透为零，则不会施加扭转摩擦。 
    它用于近似由接触表面的压缩引入的旋转摩擦。

	<b>Default:</b> 0.0
	\param[in] radius	<b>Range:</b> (0, PX_MAX_F32)
	*/
	virtual			void						setTorsionalPatchRadius(PxReal radius) = 0;

	/**
	\brief Gets torsional patch radius.     获取扭转补丁半径。

	This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
	will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
	so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
	rotational friction introduced by the compression of contacting surfaces.

	\return The torsional patch radius of the shape.
	*/
	virtual			PxReal						getTorsionalPatchRadius() const = 0;

	/**
	\brief Sets minimum torsional patch radius. 设置最小扭转补丁半径。

	This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
	that will be applied will be entirely dependent on the value of torsionalPatchRadius. 
    这定义了用于施加扭转摩擦的接触面的最小半径。 如果半径为 0，则将应用的扭转摩擦量将完全取决于 torsionalPatchRadius 的值。
	
	If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    如果半径 > 0，则无论 torsionalPatchRadius 的值或穿透量如何，都会应用一些扭转摩擦。

	<b>Default:</b> 0.0
	\param[in] radius	<b>Range:</b> (0, PX_MAX_F32)
	*/
	virtual			void						setMinTorsionalPatchRadius(PxReal radius) = 0;

	/**
	\brief Gets minimum torsional patch radius.

	This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
	that will be applied will be entirely dependent on the value of torsionalPatchRadius. 
	
	If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.

	\return The minimum torsional patch radius of the shape.
	*/
	virtual			PxReal						getMinTorsionalPatchRadius() const = 0;


/************************************************************************************************/

	/**
	\brief Sets shape flags
	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.
	\param[in] flag The shape flag to enable/disable. See #PxShapeFlag.
	\param[in] value True to set the flag. False to clear the flag specified in flag.
	<b>Default:</b> PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE
	@see PxShapeFlag getFlags()
	*/
	virtual		void					setFlag(PxShapeFlag::Enum flag, bool value) = 0;

	/**
	\brief Sets shape flags
	@see PxShapeFlag getFlags()
	*/
	virtual		void					setFlags(PxShapeFlags inFlags) = 0;

	/**
	\brief Retrieves shape flags.
	\return The values of the shape flags.
	@see PxShapeFlag setFlag()
	*/
	virtual		PxShapeFlags			getFlags() const = 0;

	/**
	\brief Returns true if the shape is exclusive to an actor.
           如果形状是actor独有的，则返回 true。
	@see PxPhysics::createShape()
	*/
	virtual		bool					isExclusive() const	= 0;

	/**
	\brief Sets a name string for the object that can be retrieved with #getName().
           为可以使用#getName() 检索的对象设置名称字符串。
	This is for debugging and is not used by the SDK.
	The string is not copied by the SDK, only the pointer is stored.
    这用于调试，SDK 不使用。
    SDK不复制字符串，只存储指针
	<b>Default:</b> NULL
	\param[in] name The name string to set the objects name to.
	@see getName()
	*/
	virtual		void					setName(const char* name)		= 0;


	/**
	\brief retrieves the name string set with setName().
	\return The name associated with the shape.
	@see setName()
	*/
	virtual		const char*				getName()			const	= 0;


	virtual		const char*				getConcreteTypeName() const	{ return "PxShape"; }

/************************************************************************************************/

				void*					userData;	//!< user can assign this to whatever, usually to create a 1:1 relationship with a user object.

protected:
	PX_INLINE							PxShape(PxBaseFlags baseFlags) : PxBase(baseFlags) {}
	PX_INLINE							PxShape(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags), userData(NULL) {}
	virtual								~PxShape() {}
	virtual		bool					isKindOf(const char* name) const { return !::strcmp("PxShape", name) || PxBase::isKindOf(name); }

};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
