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

#include "common/PxProfileZone.h"
#include "geometry/PxMeshQuery.h"
#include "PxRigidDynamic.h"
#include "foundation/PxMathUtils.h"

#include "CctCharacterController.h"
#include "CctCharacterControllerManager.h"
#include "CctSweptBox.h"
#include "CctSweptCapsule.h"
#include "CctObstacleContext.h"
#include "CmRenderOutput.h"
#include "GuIntersectionBoxBox.h"
#include "GuDistanceSegmentBox.h"
#include "PsMathUtils.h"
#include "PsFPU.h"

// PT: TODO: remove those includes.... shouldn't be allowed from here
#include "characterkinematic/PxControllerObstacles.h"	// (*)
#include "characterkinematic/PxControllerManager.h"		// (*)
#include "characterkinematic/PxControllerBehavior.h"	// (*)
#include "CctInternalStructs.h"		// (*)

//#define DEBUG_MTD
#ifdef DEBUG_MTD
	#include <stdio.h>
#endif

#define	MAX_ITER	10

using namespace physx;
using namespace Cct;
using namespace Gu;
using namespace Cm;

static const PxU32 gObstacleDebugColor = PxU32(PxDebugColor::eARGB_CYAN);
//static const PxU32 gCCTBoxDebugColor = PxU32(PxDebugColor::eARGB_YELLOW);
static const PxU32 gTBVDebugColor = PxU32(PxDebugColor::eARGB_MAGENTA);
static const bool gUsePartialUpdates = true;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE PxHitFlags getSweepHitFlags(const CCTParams& params)
{
	PxHitFlags sweepHitFlags = PxHitFlag::eDEFAULT/*|PxHitFlag::eMESH_BOTH_SIDES*/;
//	sweepHitFlags |= PxHitFlag::eASSUME_NO_INITIAL_OVERLAP;
	if(params.mPreciseSweeps)
		sweepHitFlags |= PxHitFlag::ePRECISE_SWEEP;
	return sweepHitFlags;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool shouldApplyRecoveryModule(const PxRigidActor& rigidActor)
{
	// PT: we must let the dynamic objects go through the CCT for proper 2-way interactions.
	// But we should still apply the recovery module for kinematics.

	const PxType type = rigidActor.getConcreteType();
	if(type==PxConcreteType::eRIGID_STATIC)
		return true;

	if(type!=PxConcreteType::eRIGID_DYNAMIC)
		return false;

	return static_cast<const PxRigidBody&>(rigidActor).getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const bool gUseLocalSpace = true;
static PxVec3 worldToLocal(const PxObstacle& obstacle, const PxExtendedVec3& worldPos)
{
	const PxTransform tr(toVec3(obstacle.mPos), obstacle.mRot);
	return tr.transformInv(toVec3(worldPos));
}

static PxVec3 localToWorld(const PxObstacle& obstacle, const PxVec3& localPos)
{
	const PxTransform tr(toVec3(obstacle.mPos), obstacle.mRot);
	return tr.transform(localPos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef PX_BIG_WORLDS
	typedef	PxExtendedBounds3	PxCCTBounds3;
	typedef	PxExtendedVec3		PxCCTVec3;
#else
	typedef	PxBounds3			PxCCTBounds3;
	typedef	PxVec3				PxCCTVec3;
#endif

static PX_INLINE void scale(PxCCTBounds3& b, const PxVec3& scale)
{
	PxCCTVec3 center;	getCenter(b, center);
	PxVec3 extents;		getExtents(b, extents);
	extents.x *= scale.x;
	extents.y *= scale.y;
	extents.z *= scale.z;
	setCenterExtents(b, center, extents);
}

static PX_INLINE void computeReflexionVector(PxVec3& reflected, const PxVec3& incomingDir, const PxVec3& outwardNormal)
{
	reflected = incomingDir - outwardNormal * 2.0f * (incomingDir.dot(outwardNormal));
}

static PX_INLINE void collisionResponse(PxExtendedVec3& targetPosition, const PxExtendedVec3& currentPosition, const PxVec3& currentDir, const PxVec3& hitNormal, PxF32 bump, PxF32 friction, bool normalize=false)
{
	// Compute reflect direction
	PxVec3 reflectDir;
	computeReflexionVector(reflectDir, currentDir, hitNormal);
	reflectDir.normalize();

	// Decompose it
	PxVec3 normalCompo, tangentCompo;
	Ps::decomposeVector(normalCompo, tangentCompo, reflectDir, hitNormal);

	// Compute new destination position
    const PxF32 amplitude = (targetPosition - currentPosition).magnitude();
    
	targetPosition = currentPosition;
	if(bump!=0.0f)
	{
		if(normalize)
			normalCompo.normalize();
        targetPosition += normalCompo*bump*amplitude;
	}
	if(friction!=0.0f)
	{
		if(normalize)
			tangentCompo.normalize();
        targetPosition += tangentCompo*friction*amplitude;
	}
}

static PX_INLINE void relocateBox(PxBoxGeometry& boxGeom, PxTransform& pose, const PxExtendedVec3& center, const PxVec3& extents, const PxExtendedVec3& origin, const PxQuat& quatFromUp)
{
	boxGeom.halfExtents = extents;

	pose.p.x = float(center.x - origin.x);
	pose.p.y = float(center.y - origin.y);
	pose.p.z = float(center.z - origin.z);

	pose.q = quatFromUp;
}

static PX_INLINE void relocateBox(PxBoxGeometry& boxGeom, PxTransform& pose, const TouchedUserBox& userBox)
{
	relocateBox(boxGeom, pose, userBox.mBox.center, userBox.mBox.extents, userBox.mOffset, userBox.mBox.rot);
}

static PX_INLINE void relocateBox(PxBoxGeometry& boxGeom, PxTransform& pose, const TouchedBox& box)
{
	boxGeom.halfExtents = box.mExtents;

	pose.p = box.mCenter;
	pose.q = box.mRot;
}

static PX_INLINE void relocateCapsule(
	PxCapsuleGeometry& capsuleGeom, PxTransform& pose, const SweptCapsule* sc,
	const PxQuat& quatFromUp,
	const PxExtendedVec3& center, const PxExtendedVec3& origin)
{
	capsuleGeom.radius = sc->mRadius;
	capsuleGeom.halfHeight = 0.5f * sc->mHeight;

	pose.p.x = float(center.x - origin.x);
	pose.p.y = float(center.y - origin.y);
	pose.p.z = float(center.z - origin.z);

	pose.q = quatFromUp;
}

static PX_INLINE void relocateCapsule(PxCapsuleGeometry& capsuleGeom, PxTransform& pose, const PxVec3& p0, const PxVec3& p1, PxReal radius)
{
	capsuleGeom.radius = radius;
	pose = PxTransformFromSegment(p0, p1, &capsuleGeom.halfHeight);
	if(capsuleGeom.halfHeight==0.0f)
		capsuleGeom.halfHeight = FLT_EPSILON;
}

static PX_INLINE void relocateCapsule(PxCapsuleGeometry& capsuleGeom, PxTransform& pose, const TouchedUserCapsule& userCapsule)
{
	PxVec3 p0, p1;
	p0.x = float(userCapsule.mCapsule.p0.x - userCapsule.mOffset.x);
	p0.y = float(userCapsule.mCapsule.p0.y - userCapsule.mOffset.y);
	p0.z = float(userCapsule.mCapsule.p0.z - userCapsule.mOffset.z);
	p1.x = float(userCapsule.mCapsule.p1.x - userCapsule.mOffset.x);
	p1.y = float(userCapsule.mCapsule.p1.y - userCapsule.mOffset.y);
	p1.z = float(userCapsule.mCapsule.p1.z - userCapsule.mOffset.z);

	relocateCapsule(capsuleGeom, pose, p0, p1, userCapsule.mCapsule.radius);
}

static bool SweepBoxUserBox(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eBOX);
	PX_ASSERT(geom->mType==TouchedGeomType::eUSER_BOX);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedUserBox* TC = static_cast<const TouchedUserBox*>(geom);

	PxBoxGeometry boxGeom0;
	PxTransform boxPose0;
	// To precompute
	relocateBox(boxGeom0, boxPose0, center, SB->mExtents, TC->mOffset, test->mUserParams.mQuatFromUp);

	PxBoxGeometry boxGeom1;
	PxTransform boxPose1;
	relocateBox(boxGeom1, boxPose1, *TC);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, boxGeom0, boxPose0, boxGeom1, boxPose1, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mWorldNormal		= sweepHit.normal;
	impact.mDistance		= sweepHit.distance;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.setWorldPos(sweepHit.position, TC->mOffset);
	return true;
}

static bool SweepBoxUserCapsule(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eBOX);
	PX_ASSERT(geom->mType==TouchedGeomType::eUSER_CAPSULE);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedUserCapsule* TC = static_cast<const TouchedUserCapsule*>(geom);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	// To precompute
	relocateBox(boxGeom, boxPose, center, SB->mExtents, TC->mOffset, test->mUserParams.mQuatFromUp);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	relocateCapsule(capsuleGeom, capsulePose, *TC);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, boxGeom, boxPose, capsuleGeom, capsulePose, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*{
		// ### check this
		float t;
		PxVec3 p;
		float d = gUtilLib->PxSegmentOBBSqrDist(Capsule, Box0.center, Box0.extents, Box0.rot, &t, &p);
		Box0.rot.multiply(p,p);
		impact.mWorldPos.x = p.x + Box0.center.x + TC->mOffset.x;
		impact.mWorldPos.y = p.y + Box0.center.y + TC->mOffset.y;
		impact.mWorldPos.z = p.z + Box0.center.z + TC->mOffset.z;
	}*/
	{
		impact.setWorldPos(sweepHit.position, TC->mOffset);
	}
	return true;
}

static bool sweepVolumeVsMesh(	const SweepTest* sweepTest, const TouchedMesh* touchedMesh, SweptContact& impact,
								const PxVec3& unitDir, const PxGeometry& geom, const PxTransform& pose,
								PxU32 nbTris, const PxTriangle* triangles,
								PxU32 cachedIndex)
{
	PxSweepHit sweepHit;
	if(PxMeshQuery::sweep(unitDir, impact.mDistance, geom, pose, nbTris, triangles, sweepHit, getSweepHitFlags(sweepTest->mUserParams), &cachedIndex))
	{
		if(sweepHit.distance >= impact.mDistance)
			return false;

		impact.mDistance	= sweepHit.distance;
		impact.mWorldNormal	= sweepHit.normal;
		impact.setWorldPos(sweepHit.position, touchedMesh->mOffset);

		// Returned index is only between 0 and nbTris, i.e. it indexes the array of cached triangles, not the original mesh.
		PX_ASSERT(sweepHit.faceIndex < nbTris);
		sweepTest->mCachedTriIndex[sweepTest->mCachedTriIndexIndex] = sweepHit.faceIndex;

		// The CCT loop will use the index from the start of the cache...
		impact.mInternalIndex = sweepHit.faceIndex + touchedMesh->mIndexWorldTriangles;
		const PxU32* triangleIndices = &sweepTest->mTriangleIndices[touchedMesh->mIndexWorldTriangles];
		impact.mTriangleIndex = triangleIndices[sweepHit.faceIndex];
		return true;
	}
	return false;
}

static bool SweepBoxMesh(const SweepTest* sweep_test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eBOX);
	PX_ASSERT(geom->mType==TouchedGeomType::eMESH);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedMesh* TM = static_cast<const TouchedMesh*>(geom);

	PxU32 nbTris = TM->mNbTris;
	if(!nbTris)
		return false;

	// Fetch triangle data for current mesh (the stream may contain triangles from multiple meshes)
	const PxTriangle* T = &sweep_test->mWorldTriangles.getTriangle(TM->mIndexWorldTriangles);

	// PT: this only really works when the CCT collides with a single mesh, but that's the most common case. When it doesn't, there's just no speedup but it still works.
	PxU32 CachedIndex = sweep_test->mCachedTriIndex[sweep_test->mCachedTriIndexIndex];
	if(CachedIndex>=nbTris)
		CachedIndex=0;

	PxBoxGeometry boxGeom;
	boxGeom.halfExtents = SB->mExtents;
	PxTransform boxPose(PxVec3(float(center.x - TM->mOffset.x), float(center.y - TM->mOffset.y), float(center.z - TM->mOffset.z)), sweep_test->mUserParams.mQuatFromUp);  // Precompute
	return sweepVolumeVsMesh(sweep_test, TM, impact, dir, boxGeom, boxPose, nbTris, T, CachedIndex);
}

static bool SweepCapsuleMesh(
	const SweepTest* sweep_test, const SweptVolume* volume, const TouchedGeom* geom,
	const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	PX_ASSERT(geom->mType==TouchedGeomType::eMESH);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedMesh* TM = static_cast<const TouchedMesh*>(geom);

	PxU32 nbTris = TM->mNbTris;
	if(!nbTris)
		return false;

	// Fetch triangle data for current mesh (the stream may contain triangles from multiple meshes)
	const PxTriangle* T = &sweep_test->mWorldTriangles.getTriangle(TM->mIndexWorldTriangles);

	// PT: this only really works when the CCT collides with a single mesh, but that's the most common case.
	// When it doesn't, there's just no speedup but it still works.
	PxU32 CachedIndex = sweep_test->mCachedTriIndex[sweep_test->mCachedTriIndexIndex];
	if(CachedIndex>=nbTris)
		CachedIndex=0;

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	relocateCapsule(capsuleGeom, capsulePose, SC, sweep_test->mUserParams.mQuatFromUp, center, TM->mOffset);

	return sweepVolumeVsMesh(sweep_test, TM, impact, dir, capsuleGeom, capsulePose, nbTris, T, CachedIndex);
}

static bool SweepBoxBox(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eBOX);
	PX_ASSERT(geom->mType==TouchedGeomType::eBOX);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedBox* TB = static_cast<const TouchedBox*>(geom);

	PxBoxGeometry boxGeom0;
	PxTransform boxPose0;
	// To precompute
	relocateBox(boxGeom0, boxPose0, center, SB->mExtents, TB->mOffset, test->mUserParams.mQuatFromUp);

	PxBoxGeometry boxGeom1;
	PxTransform boxPose1;
	relocateBox(boxGeom1, boxPose1, *TB);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, boxGeom0, boxPose0, boxGeom1, boxPose1, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mWorldNormal		= sweepHit.normal;
	impact.mDistance		= sweepHit.distance;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.setWorldPos(sweepHit.position, TB->mOffset);
	return true;
}

static bool SweepBoxSphere(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eBOX);
	PX_ASSERT(geom->mType==TouchedGeomType::eSPHERE);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedSphere* TS = static_cast<const TouchedSphere*>(geom);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	// To precompute
	relocateBox(boxGeom, boxPose, center, SB->mExtents, TS->mOffset, test->mUserParams.mQuatFromUp);

	PxSphereGeometry sphereGeom;
	sphereGeom.radius = TS->mRadius;
	PxTransform spherePose;
	spherePose.p = TS->mCenter;
	spherePose.q = PxQuat(PxIdentity);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, boxGeom, boxPose, sphereGeom, spherePose, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*
	{
		// The sweep test doesn't compute the impact point automatically, so we have to do it here.
		PxVec3 NewSphereCenter = TS->mSphere.center - d * dir;
		PxVec3 Closest;
		gUtilLib->PxPointOBBSqrDist(NewSphereCenter, Box0.center, Box0.extents, Box0.rot, &Closest);
		// Compute point on the box, after sweep
		Box0.rot.multiply(Closest, Closest);
		impact.mWorldPos.x = TS->mOffset.x + Closest.x + Box0.center.x + d * dir.x;
		impact.mWorldPos.y = TS->mOffset.y + Closest.y + Box0.center.y + d * dir.y;
		impact.mWorldPos.z = TS->mOffset.z + Closest.z + Box0.center.z + d * dir.z;

		impact.mWorldNormal = -impact.mWorldNormal;
	}*/
	{
		impact.setWorldPos(sweepHit.position, TS->mOffset);
	}
	return true;
}

static bool SweepBoxCapsule(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eBOX);
	PX_ASSERT(geom->mType==TouchedGeomType::eCAPSULE);
	const SweptBox* SB = static_cast<const SweptBox*>(volume);
	const TouchedCapsule* TC = static_cast<const TouchedCapsule*>(geom);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	// To precompute
	relocateBox(boxGeom, boxPose, center, SB->mExtents, TC->mOffset, test->mUserParams.mQuatFromUp);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	relocateCapsule(capsuleGeom, capsulePose, TC->mP0, TC->mP1, TC->mRadius);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, boxGeom, boxPose, capsuleGeom, capsulePose, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*{
		float t;
		PxVec3 p;
		float d = gUtilLib->PxSegmentOBBSqrDist(TC->mCapsule, Box0.center, Box0.extents, Box0.rot, &t, &p);
		Box0.rot.multiply(p,p);
		impact.mWorldPos.x = p.x + Box0.center.x + TC->mOffset.x;
		impact.mWorldPos.y = p.y + Box0.center.y + TC->mOffset.y;
		impact.mWorldPos.z = p.z + Box0.center.z + TC->mOffset.z;
	}*/
	{
		impact.setWorldPos(sweepHit.position, TC->mOffset);
	}
	return true;
}

static bool SweepCapsuleBox(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	PX_ASSERT(geom->mType==TouchedGeomType::eBOX);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedBox* TB = static_cast<const TouchedBox*>(geom);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	relocateCapsule(capsuleGeom, capsulePose, SC, test->mUserParams.mQuatFromUp, center, TB->mOffset);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	// To precompute
	relocateBox(boxGeom, boxPose, *TB);

	// The box and capsule coordinates are relative to the center of the cached bounding box
	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, capsuleGeom, capsulePose, boxGeom, boxPose, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;

	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*{
		float t;
		PxVec3 p;
		float d = gUtilLib->PxSegmentOBBSqrDist(Capsule, TB->mBox.center, TB->mBox.extents, TB->mBox.rot, &t, &p);
		TB->mBox.rot.multiply(p,p);
		p += TB->mBox.center;
		impact.mWorldPos.x = p.x + TB->mOffset.x;
		impact.mWorldPos.y = p.y + TB->mOffset.y;
		impact.mWorldPos.z = p.z + TB->mOffset.z;
	}*/
	{
		impact.setWorldPos(sweepHit.position, TB->mOffset);
	}
	return true;
}

static bool SweepCapsuleSphere(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	PX_ASSERT(geom->mType==TouchedGeomType::eSPHERE);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedSphere* TS = static_cast<const TouchedSphere*>(geom);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	relocateCapsule(capsuleGeom, capsulePose, SC, test->mUserParams.mQuatFromUp, center, TS->mOffset);

	PxSphereGeometry sphereGeom;
	sphereGeom.radius = TS->mRadius;
	PxTransform spherePose;
	spherePose.p = TS->mCenter;
	spherePose.q = PxQuat(PxIdentity);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, capsuleGeom, capsulePose, sphereGeom, spherePose, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.setWorldPos(sweepHit.position, TS->mOffset);
	return true;
}

static bool SweepCapsuleCapsule(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	PX_ASSERT(geom->mType==TouchedGeomType::eCAPSULE);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedCapsule* TC = static_cast<const TouchedCapsule*>(geom);

	PxCapsuleGeometry capsuleGeom0;
	PxTransform capsulePose0;
	relocateCapsule(capsuleGeom0, capsulePose0, SC, test->mUserParams.mQuatFromUp, center, TC->mOffset);

	PxCapsuleGeometry capsuleGeom1;
	PxTransform capsulePose1;
	relocateCapsule(capsuleGeom1, capsulePose1, TC->mP0, TC->mP1, TC->mRadius);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, capsuleGeom0, capsulePose0, capsuleGeom1, capsulePose1, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.setWorldPos(sweepHit.position, TC->mOffset);
	return true;
}

static bool SweepCapsuleUserCapsule(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	PX_ASSERT(geom->mType==TouchedGeomType::eUSER_CAPSULE);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedUserCapsule* TC = static_cast<const TouchedUserCapsule*>(geom);

	PxCapsuleGeometry capsuleGeom0;
	PxTransform capsulePose0;
	relocateCapsule(capsuleGeom0, capsulePose0, SC, test->mUserParams.mQuatFromUp, center, TC->mOffset);

	PxCapsuleGeometry capsuleGeom1;
	PxTransform capsulePose1;
	relocateCapsule(capsuleGeom1, capsulePose1, *TC);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, capsuleGeom0, capsulePose0, capsuleGeom1, capsulePose1, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.setWorldPos(sweepHit.position, TC->mOffset);
	return true;
}

static bool SweepCapsuleUserBox(const SweepTest* test, const SweptVolume* volume, const TouchedGeom* geom, const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact)
{
	PX_ASSERT(volume->getType()==SweptVolumeType::eCAPSULE);
	PX_ASSERT(geom->mType==TouchedGeomType::eUSER_BOX);
	const SweptCapsule* SC = static_cast<const SweptCapsule*>(volume);
	const TouchedUserBox* TB = static_cast<const TouchedUserBox*>(geom);

	PxCapsuleGeometry capsuleGeom;
	PxTransform capsulePose;
	relocateCapsule(capsuleGeom, capsulePose, SC, test->mUserParams.mQuatFromUp, center, TB->mOffset);

	PxBoxGeometry boxGeom;
	PxTransform boxPose;
	relocateBox(boxGeom, boxPose, *TB);

	PxSweepHit sweepHit;
	if(!PxGeometryQuery::sweep(dir, impact.mDistance, capsuleGeom, capsulePose, boxGeom, boxPose, sweepHit, getSweepHitFlags(test->mUserParams)))
		return false;

	if(sweepHit.distance >= impact.mDistance)
		return false;

	impact.mDistance		= sweepHit.distance;
	impact.mWorldNormal		= sweepHit.normal;
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;

	//TO CHECK: Investigate whether any significant performance improvement can be achieved through
	//          making the impact point computation optional in the sweep calls and compute it later
	/*{
		// ### check this
		float t;
		PxVec3 p;
		float d = gUtilLib->PxSegmentOBBSqrDist(Capsule, Box.center, Box.extents, Box.rot, &t, &p);
		p += Box.center;
		impact.mWorldPos.x = p.x + TB->mOffset.x;
		impact.mWorldPos.y = p.y + TB->mOffset.y;
		impact.mWorldPos.z = p.z + TB->mOffset.z;
	}*/
	{
		impact.setWorldPos(sweepHit.position, TB->mOffset);
	}
	return true;
}

typedef bool (*SweepFunc) (const SweepTest*, const SweptVolume*, const TouchedGeom*, const PxExtendedVec3&, const PxVec3&, SweptContact&);

static SweepFunc gSweepMap[SweptVolumeType::eLAST][TouchedGeomType::eLAST] = {
	// Box funcs
	{
	SweepBoxUserBox,
	SweepBoxUserCapsule,
	SweepBoxMesh,
	SweepBoxBox,
	SweepBoxSphere,
	SweepBoxCapsule
	},

	// Capsule funcs
	{
	SweepCapsuleUserBox,
	SweepCapsuleUserCapsule,
	SweepCapsuleMesh,
	SweepCapsuleBox,
	SweepCapsuleSphere,
	SweepCapsuleCapsule
	}
};

PX_COMPILE_TIME_ASSERT(sizeof(gSweepMap)==SweptVolumeType::eLAST*TouchedGeomType::eLAST*sizeof(SweepFunc));

static const PxU32 GeomSizes[] =
{
	sizeof(TouchedUserBox),
	sizeof(TouchedUserCapsule),
	sizeof(TouchedMesh),
	sizeof(TouchedBox),
	sizeof(TouchedSphere),
	sizeof(TouchedCapsule),
};

static const TouchedGeom* CollideGeoms(
	const SweepTest* sweep_test, const SweptVolume& volume, const IntArray& geom_stream,
	const PxExtendedVec3& center, const PxVec3& dir, SweptContact& impact, bool discardInitialOverlap)
{
	impact.mInternalIndex	= PX_INVALID_U32;
	impact.mTriangleIndex	= PX_INVALID_U32;
	impact.mGeom			= NULL;

	const PxU32* Data = geom_stream.begin();
	const PxU32* Last = geom_stream.end();
	while(Data!=Last)
	{
		const TouchedGeom* CurrentGeom = reinterpret_cast<const TouchedGeom*>(Data);

		SweepFunc ST = gSweepMap[volume.getType()][CurrentGeom->mType];
		if(ST)
		{
			SweptContact C;
			C.mDistance			= impact.mDistance;	// Initialize with current best distance
			C.mInternalIndex	= PX_INVALID_U32;
			C.mTriangleIndex	= PX_INVALID_U32;
			if((ST)(sweep_test, &volume, CurrentGeom, center, dir, C))
			{
				if(C.mDistance==0.0f)
				{
					if(!discardInitialOverlap)
					{
						if(CurrentGeom->mType==TouchedGeomType::eUSER_BOX || CurrentGeom->mType==TouchedGeomType::eUSER_CAPSULE)
						{
						}
						else
						{
							const PxRigidActor* touchedActor = CurrentGeom->mActor;
							PX_ASSERT(touchedActor);

							if(shouldApplyRecoveryModule(*touchedActor))
							{
								impact = C;
								impact.mGeom = const_cast<TouchedGeom*>(CurrentGeom);
								return CurrentGeom;
							}
						}
					}
				}
/*				else
				if(discardInitialOverlap && C.mDistance==0.0f)
				{
					// PT: we previously used eINITIAL_OVERLAP without eINITIAL_OVERLAP_KEEP, i.e. initially overlapping shapes got ignored.
					// So we replicate this behavior here.
				}*/
				else if(C.mDistance<impact.mDistance)
				{
					impact = C;
					impact.mGeom = const_cast<TouchedGeom*>(CurrentGeom);
					if(C.mDistance <= 0.0f)	// there is no point testing for closer hits
						return CurrentGeom;	// since we are touching a shape already
				}
			}
		}

		const PxU8* ptr = reinterpret_cast<const PxU8*>(Data);
		ptr += GeomSizes[CurrentGeom->mType];
		Data = reinterpret_cast<const PxU32*>(ptr);
	}
	return impact.mGeom;
}

static PxVec3 computeMTD(const SweepTest* sweep_test, const SweptVolume& volume, const IntArray& geom_stream, const PxExtendedVec3& center, float contactOffset)
{
	PxVec3 p = toVec3(center);

//	contactOffset += 0.01f;

	const PxU32 maxIter = 4;
	PxU32 nbIter = 0;
	bool isValid = true;
	while(isValid && nbIter<maxIter)
	{
		const PxU32* Data = geom_stream.begin();
		const PxU32* Last = geom_stream.end();
		while(Data!=Last)
		{
			const TouchedGeom* CurrentGeom = reinterpret_cast<const TouchedGeom*>(Data);

			if(CurrentGeom->mType==TouchedGeomType::eUSER_BOX || CurrentGeom->mType==TouchedGeomType::eUSER_CAPSULE)
			{
			}
			else
			{
				const PxRigidActor* touchedActor = CurrentGeom->mActor;
				PX_ASSERT(touchedActor);

				if(shouldApplyRecoveryModule(*touchedActor))
				{
					const PxShape* touchedShape = reinterpret_cast<const PxShape*>(CurrentGeom->mTGUserData);
					PX_ASSERT(touchedShape);

					const PxGeometryHolder gh = touchedShape->getGeometry();
					const PxTransform globalPose = getShapeGlobalPose(*touchedShape, *touchedActor);

					PxVec3 mtd;
					PxF32 depth;

					const PxTransform volumePose(p, sweep_test->mUserParams.mQuatFromUp);
					if(volume.getType()==SweptVolumeType::eCAPSULE)
					{
						const SweptCapsule& sc = static_cast<const SweptCapsule&>(volume);
						const PxCapsuleGeometry capsuleGeom(sc.mRadius+contactOffset, sc.mHeight*0.5f);
						isValid = PxGeometryQuery::computePenetration(mtd, depth, capsuleGeom, volumePose, gh.any(), globalPose);
					}
					else
					{
						PX_ASSERT(volume.getType()==SweptVolumeType::eBOX);
						const SweptBox& sb = static_cast<const SweptBox&>(volume);
						const PxBoxGeometry boxGeom(sb.mExtents+PxVec3(contactOffset));
						isValid = PxGeometryQuery::computePenetration(mtd, depth, boxGeom, volumePose, gh.any(), globalPose);
					}

					if(isValid)
					{
						nbIter++;
						PX_ASSERT(depth>=0.0f);
						PX_ASSERT(mtd.isFinite());
						PX_ASSERT(PxIsFinite(depth));
#ifdef DEBUG_MTD
						PX_ASSERT(depth<=1.0f);
						if(depth>1.0f || !mtd.isFinite() || !PxIsFinite(depth))
						{
							int stop=1;
							(void)stop;
						}
						printf("Depth: %f\n", depth);
						printf("mtd: %f %f %f\n", mtd.x, mtd.y, mtd.z);
#endif
						p += mtd * depth;
					}
				}
			}

			const PxU8* ptr = reinterpret_cast<const PxU8*>(Data);
			ptr += GeomSizes[CurrentGeom->mType];
			Data = reinterpret_cast<const PxU32*>(ptr);
		}
	}
	return p;
}



static bool ParseGeomStream(const void* object, const IntArray& geom_stream)
{
	const PxU32* Data = geom_stream.begin();
	const PxU32* Last = geom_stream.end();
	while(Data!=Last)
	{
		const TouchedGeom* CurrentGeom = reinterpret_cast<const TouchedGeom*>(Data);
		if(CurrentGeom->mTGUserData==object)
			return true;

		const PxU8* ptr = reinterpret_cast<const PxU8*>(Data);
		ptr += GeomSizes[CurrentGeom->mType];
		Data = reinterpret_cast<const PxU32*>(ptr);
	}
	return false;
}

CCTParams::CCTParams() :
	mNonWalkableMode						(PxControllerNonWalkableMode::ePREVENT_CLIMBING),
	mQuatFromUp								(PxQuat(PxIdentity)),
	mUpDirection							(PxVec3(0.0f)),
	mSlopeLimit								(0.0f),
	mContactOffset							(0.0f),
	mStepOffset								(0.0f),
	mInvisibleWallHeight					(0.0f),
	mMaxJumpHeight							(0.0f),
	mMaxEdgeLength2							(0.0f),
	mTessellation							(false),
	mHandleSlope							(false),
	mOverlapRecovery						(false),
	mPreciseSweeps							(true),
	mPreventVerticalSlidingAgainstCeiling	(false)
{
}

SweepTest::SweepTest(bool registerDeletionListener) :
	mRenderBuffer		(NULL),
	mRenderFlags		(0),
	mTriangleIndices	(PX_DEBUG_EXP("sweepTestTriangleIndices")),
	mGeomStream			(PX_DEBUG_EXP("sweepTestStream")),
	mTouchedShape		(registerDeletionListener),
	mTouchedActor		(registerDeletionListener),
	mSQTimeStamp		(0xffffffff),
	mNbFullUpdates		(0),
	mNbPartialUpdates	(0),
	mNbTessellation		(0),
	mNbIterations		(0),
	mFlags				(0),
	mRegisterDeletionListener(registerDeletionListener),
	mCctManager			(NULL)
{
	mCacheBounds.setEmpty();
	mCachedTriIndexIndex	= 0;
	mCachedTriIndex[0] = mCachedTriIndex[1] = mCachedTriIndex[2] = 0;
	mNbCachedStatic = 0;
	mNbCachedT		= 0;

	mTouchedObstacleHandle	= INVALID_OBSTACLE_HANDLE;
	mTouchedPos					= PxVec3(0);
	mTouchedPosShape_Local		= PxVec3(0);
	mTouchedPosShape_World		= PxVec3(0);
	mTouchedPosObstacle_Local	= PxVec3(0);
	mTouchedPosObstacle_World	= PxVec3(0);

//	mVolumeGrowth	= 1.2f;	// Must be >1.0f and not too big
	mVolumeGrowth	= 1.5f;	// Must be >1.0f and not too big
//	mVolumeGrowth	= 2.0f;	// Must be >1.0f and not too big

	mContactNormalDownPass = PxVec3(0.0f);
	mContactNormalSidePass = PxVec3(0.0f);
	mTouchedTriMin = 0.0f;
	mTouchedTriMax = 0.0f;
}


SweepTest::~SweepTest()
{
	// set the TouchedObject to NULL so we unregister the actor/shape
	mTouchedShape = NULL;
	mTouchedActor = NULL;
}

void SweepTest::voidTestCache()
{
	mTouchedShape = NULL;
	mTouchedActor = NULL;
	mCacheBounds.setEmpty();
	mTouchedObstacleHandle	= INVALID_OBSTACLE_HANDLE;
}

void SweepTest::onRelease(const PxBase& observed)
{	
	if (mTouchedActor == &observed)
	{
		mTouchedShape = NULL;
		mTouchedActor = NULL;
		return;
	}

	if(ParseGeomStream(&observed, mGeomStream))
		mCacheBounds.setEmpty();

	if (mTouchedShape == &observed)
		mTouchedShape = NULL;
}

void SweepTest::updateCachedShapesRegistration(PxU32 startIndex, bool unregister)
{
	if(!mRegisterDeletionListener)
		return;

	if(!mGeomStream.size() || startIndex == mGeomStream.size())
		return;

	PX_ASSERT(startIndex <= mGeomStream.size());

	const PxU32* data = &mGeomStream[startIndex];
	const PxU32* last = mGeomStream.end();
	while (data != last)
	{
		const TouchedGeom* CurrentGeom = reinterpret_cast<const TouchedGeom*>(data);
		if (CurrentGeom->mActor)
		{
			if(unregister)
				mCctManager->unregisterObservedObject(reinterpret_cast<const PxBase*>(CurrentGeom->mTGUserData));
			else
				mCctManager->registerObservedObject(reinterpret_cast<const PxBase*>(CurrentGeom->mTGUserData));
		}
		else
		{
			// we can early exit, the rest of the data are user obstacles
			return;
		}

		const PxU8* ptr = reinterpret_cast<const PxU8*>(data);
		ptr += GeomSizes[CurrentGeom->mType];
		data = reinterpret_cast<const PxU32*>(ptr);
	}
}

void SweepTest::onObstacleAdded(ObstacleHandle index, const PxObstacleContext* context, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance )
{
	if(mTouchedObstacleHandle != INVALID_OBSTACLE_HANDLE)
	{
		// check if new obstacle is closer
		const ObstacleContext* obstContext = static_cast<const ObstacleContext*> (context);
		PxRaycastHit obstacleHit;
		const PxObstacle* obst = obstContext->raycastSingle(obstacleHit,index,origin,unitDir,distance);

		if(obst && (obstacleHit.position.dot(unitDir))<(mTouchedPosObstacle_World.dot(unitDir)))
		{
			PX_ASSERT(obstacleHit.distance<=distance);
			mTouchedObstacleHandle = index;
			if(!gUseLocalSpace)
			{
				mTouchedPos = toVec3(obst->mPos);
			}
			else
			{
				mTouchedPosObstacle_World = obstacleHit.position;
				mTouchedPosObstacle_Local = worldToLocal(*obst, PxExtendedVec3(PxExtended(obstacleHit.position.x),PxExtended(obstacleHit.position.y),PxExtended(obstacleHit.position.z)));
			}
		}
	}
}

void SweepTest::onObstacleRemoved(ObstacleHandle index)
{
	if(index == mTouchedObstacleHandle)
	{
		mTouchedObstacleHandle = INVALID_OBSTACLE_HANDLE;
	}
}

void SweepTest::onObstacleUpdated(ObstacleHandle index, const PxObstacleContext* context, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance)
{
	if(index == mTouchedObstacleHandle)
	{
		// check if updated obstacle is still closest
		const ObstacleContext* obstContext = static_cast<const ObstacleContext*> (context);
		PxRaycastHit obstacleHit;
		ObstacleHandle closestHandle = INVALID_OBSTACLE_HANDLE;
		const PxObstacle* obst = obstContext->raycastSingle(obstacleHit,origin,unitDir,distance,closestHandle);

		if(mTouchedObstacleHandle == closestHandle)
			return;

		if(obst)
		{
			PX_ASSERT(obstacleHit.distance<=distance);
			mTouchedObstacleHandle = closestHandle;
			if(!gUseLocalSpace)
			{
				mTouchedPos = toVec3(obst->mPos);
			}
			else
			{
				mTouchedPosObstacle_World = obstacleHit.position;
				mTouchedPosObstacle_Local = worldToLocal(*obst, PxExtendedVec3(PxExtended(obstacleHit.position.x),PxExtended(obstacleHit.position.y),PxExtended(obstacleHit.position.z)));
			}
		}
	}
}

void SweepTest::onOriginShift(const PxVec3& shift)
{
	mCacheBounds.minimum -= shift;
	mCacheBounds.maximum -= shift;

	if(mTouchedShape)
	{
		const PxRigidActor* rigidActor = mTouchedActor.get();
		if(rigidActor->getConcreteType() != PxConcreteType::eRIGID_STATIC)
		{
			mTouchedPosShape_World -= shift;
		}
	}
	else if (mTouchedObstacleHandle != INVALID_OBSTACLE_HANDLE)
	{
		if(!gUseLocalSpace)
		{
			mTouchedPos -= shift;
		}
		else
		{
			mTouchedPosObstacle_World -= shift;
		}
	}

	// adjust cache
	PxU32* data = mGeomStream.begin();
	PxU32* last = mGeomStream.end();
	while(data != last)
	{
		TouchedGeom* currentGeom = reinterpret_cast<TouchedGeom*>(data);

		currentGeom->mOffset -= shift;

		PxU8* ptr = reinterpret_cast<PxU8*>(data);
		ptr += GeomSizes[currentGeom->mType];
		data = reinterpret_cast<PxU32*>(ptr);
	}
}

static PxBounds3 getBounds3(const PxExtendedBounds3& extended)
{
	return PxBounds3(toVec3(extended.minimum), toVec3(extended.maximum));	// LOSS OF ACCURACY
}

// PT: finds both touched CCTs and touched user-defined obstacles
void SweepTest::findTouchedObstacles(const UserObstacles& userObstacles, const PxExtendedBounds3& worldBox)
{
	PxExtendedVec3 Origin;	// Will be TouchedGeom::mOffset
	getCenter(worldBox, Origin);

	{
		const PxU32 nbBoxes = userObstacles.mNbBoxes;
		const PxExtendedBox* boxes = userObstacles.mBoxes;
		const void** boxUserData = userObstacles.mBoxUserData;

		const PxBounds3 singlePrecisionWorldBox = getBounds3(worldBox);

		// Find touched boxes, i.e. other box controllers
		for(PxU32 i=0;i<nbBoxes;i++)
		{
			const Gu::Box obb(
				toVec3(boxes[i].center),	// LOSS OF ACCURACY
				boxes[i].extents,
				PxMat33(boxes[i].rot));	// #### PT: TODO: useless conversion here

			if(!Gu::intersectOBBAABB(obb, singlePrecisionWorldBox))
				continue;

			TouchedUserBox* UserBox = reinterpret_cast<TouchedUserBox*>(reserveContainerMemory(mGeomStream, sizeof(TouchedUserBox)/sizeof(PxU32)));
			UserBox->mType			= TouchedGeomType::eUSER_BOX;
			UserBox->mTGUserData	= boxUserData[i];
			UserBox->mActor			= NULL;
			UserBox->mOffset		= Origin;
			UserBox->mBox			= boxes[i];
		}
	}

	{
		// Find touched capsules, i.e. other capsule controllers
		const PxU32 nbCapsules = userObstacles.mNbCapsules;
		const PxExtendedCapsule* capsules = userObstacles.mCapsules;
		const void** capsuleUserData = userObstacles.mCapsuleUserData;

		PxExtendedVec3 Center;
		PxVec3 Extents;
		getCenter(worldBox, Center);
		getExtents(worldBox, Extents);

		for(PxU32 i=0;i<nbCapsules;i++)
		{
			// PT: do a quick AABB check first, to avoid calling the SDK too much
			const PxF32 r = capsules[i].radius;
			const PxExtended capMinx = PxMin(capsules[i].p0.x, capsules[i].p1.x);
			const PxExtended capMaxx = PxMax(capsules[i].p0.x, capsules[i].p1.x);
			if((capMinx - PxExtended(r) > worldBox.maximum.x) || (worldBox.minimum.x > capMaxx + PxExtended(r))) continue;

			const PxExtended capMiny = PxMin(capsules[i].p0.y, capsules[i].p1.y);
			const PxExtended capMaxy = PxMax(capsules[i].p0.y, capsules[i].p1.y);
			if((capMiny - PxExtended(r) > worldBox.maximum.y) || (worldBox.minimum.y > capMaxy + PxExtended(r))) continue;

			const PxExtended capMinz = PxMin(capsules[i].p0.z, capsules[i].p1.z);
			const PxExtended capMaxz = PxMax(capsules[i].p0.z, capsules[i].p1.z);
			if((capMinz - PxExtended(r) > worldBox.maximum.z) || (worldBox.minimum.z > capMaxz + PxExtended(r))) continue;

			// PT: more accurate capsule-box test. Not strictly necessary but worth doing if available
			const PxReal d2 = Gu::distanceSegmentBoxSquared(toVec3(capsules[i].p0), toVec3(capsules[i].p1), toVec3(Center), Extents, PxMat33(PxIdentity));
			if(d2>r*r)
				continue;

			TouchedUserCapsule* UserCapsule = reinterpret_cast<TouchedUserCapsule*>(reserveContainerMemory(mGeomStream, sizeof(TouchedUserCapsule)/sizeof(PxU32)));
			UserCapsule->mType			= TouchedGeomType::eUSER_CAPSULE;
			UserCapsule->mTGUserData	= capsuleUserData[i];
			UserCapsule->mActor			= NULL;
			UserCapsule->mOffset		= Origin;
			UserCapsule->mCapsule		= capsules[i];
		}
	}
}

void SweepTest::updateTouchedGeoms(	const InternalCBData_FindTouchedGeom* userData, const UserObstacles& userObstacles,
									const PxExtendedBounds3& worldTemporalBox, const PxControllerFilters& filters, const PxVec3& sideVector)
{
	/*
	- if this is the first iteration (new frame) we have to redo the dynamic objects & the CCTs. The static objects can
	be cached.
	- if this is not, we can cache everything
	*/

	// PT: using "worldTemporalBox" instead of "mCacheBounds" seems to produce TTP 6207
//#define DYNAMIC_BOX	worldTemporalBox
#define DYNAMIC_BOX	mCacheBounds

	bool newCachedBox = false;

	CCTFilter filter;
	filter.mFilterData		= filters.mFilterData;
	filter.mFilterCallback	= filters.mFilterCallback;
	filter.mPreFilter		= filters.mFilterFlags & PxQueryFlag::ePREFILTER;
	filter.mPostFilter		= filters.mFilterFlags & PxQueryFlag::ePOSTFILTER;

	// PT: detect changes to the static pruning structure
	bool sceneHasChanged = false;
	{
		const PxU32 currentTimestamp = getSceneTimestamp(userData);
		if(currentTimestamp!=mSQTimeStamp)
		{
			mSQTimeStamp = currentTimestamp;
			sceneHasChanged = true;
		}
	}

	// If the input box is inside the cached box, nothing to do
	if(gUsePartialUpdates && !sceneHasChanged && worldTemporalBox.isInside(mCacheBounds))
	{
		//printf("CACHEIN%d\n", mFirstUpdate);
		if(mFlags & STF_FIRST_UPDATE)
		{
			mFlags &= ~STF_FIRST_UPDATE;

			// Only redo the dynamic
			updateCachedShapesRegistration(mNbCachedStatic, true);
			mGeomStream.forceSize_Unsafe(mNbCachedStatic);
			mWorldTriangles.forceSize_Unsafe(mNbCachedT);
			mTriangleIndices.forceSize_Unsafe(mNbCachedT);			

			filter.mStaticShapes	= false;
			if(filters.mFilterFlags & PxQueryFlag::eDYNAMIC)
				filter.mDynamicShapes	= true;
			findTouchedGeometry(userData, DYNAMIC_BOX, mWorldTriangles, mTriangleIndices, mGeomStream, filter, mUserParams, mNbTessellation);
			updateCachedShapesRegistration(mNbCachedStatic, false);

			findTouchedObstacles(userObstacles, DYNAMIC_BOX);

			mNbPartialUpdates++;
		}
	}
	else
	{
		//printf("CACHEOUTNS=%d\n", mNbCachedStatic);
		newCachedBox = true;

		// Cache BV used for the query
		mCacheBounds = worldTemporalBox;

		// Grow the volume a bit. The temporal box here doesn't take sliding & collision response into account.
		// In bad cases it is possible to eventually touch a portion of space not covered by this volume. Just
		// in case, we grow the initial volume slightly. Then, additional tests are performed within the loop
		// to make sure the TBV is always correct. There's a tradeoff between the original (artificial) growth
		// of the volume, and the number of TBV recomputations performed at runtime...
		scale(mCacheBounds, PxVec3(mVolumeGrowth));
//		scale(mCacheBounds, PxVec3(mVolumeGrowth, 1.0f, mVolumeGrowth));

		if(1 && !sideVector.isZero())
		{
			const PxVec3 sn = sideVector.getNormalized();
			float dp0 = PxAbs((worldTemporalBox.maximum - worldTemporalBox.minimum).dot(sn));
			float dp1 = PxAbs((mCacheBounds.maximum - mCacheBounds.minimum).dot(sn));
			dp1 -= dp0;
			dp1 *= 0.5f * 0.9f;
			const PxVec3 offset = sn * dp1;
//			printf("%f %f %f\n", offset.x, offset.y, offset.z);
			mCacheBounds.minimum += offset;
			mCacheBounds.maximum += offset;
			add(mCacheBounds, worldTemporalBox);
			PX_ASSERT(worldTemporalBox.isInside(mCacheBounds));
		}

		updateCachedShapesRegistration(0, true);

		// Gather triangles touched by this box. This covers multiple meshes.
		mWorldTriangles.clear();
		mTriangleIndices.clear();
		mGeomStream.clear();
//		mWorldTriangles.reset();
//		mTriangleIndices.reset();
//		mGeomStream.reset();

		mCachedTriIndexIndex = 0;
		mCachedTriIndex[0] = mCachedTriIndex[1] = mCachedTriIndex[2] = 0;

		mNbFullUpdates++;

		if(filters.mFilterFlags & PxQueryFlag::eSTATIC)
			filter.mStaticShapes	= true;
		filter.mDynamicShapes	= false;
		findTouchedGeometry(userData, mCacheBounds, mWorldTriangles, mTriangleIndices, mGeomStream, filter, mUserParams, mNbTessellation);

		mNbCachedStatic = mGeomStream.size();
		mNbCachedT = mWorldTriangles.size();
		PX_ASSERT(mTriangleIndices.size()==mNbCachedT);

		filter.mStaticShapes	= false;
		if(filters.mFilterFlags & PxQueryFlag::eDYNAMIC)
			filter.mDynamicShapes	= true;
		findTouchedGeometry(userData, DYNAMIC_BOX, mWorldTriangles, mTriangleIndices, mGeomStream, filter, mUserParams, mNbTessellation);
		// We can't early exit when no tris are touched since we also have to handle the boxes
		updateCachedShapesRegistration(0, false);

		findTouchedObstacles(userObstacles, DYNAMIC_BOX);

		mFlags &= ~STF_FIRST_UPDATE;
		//printf("CACHEOUTNSDONE=%d\n", mNbCachedStatic);
	}

	if(mRenderBuffer)
	{
		// PT: worldTemporalBox = temporal BV for this frame
		RenderOutput out(*mRenderBuffer);

		if(mRenderFlags & PxControllerDebugRenderFlag::eTEMPORAL_BV)
		{
			out << gTBVDebugColor;
			out << DebugBox(getBounds3(worldTemporalBox));
		}

		if(mRenderFlags & PxControllerDebugRenderFlag::eCACHED_BV)
		{
			if(newCachedBox)
				out << PxU32(PxDebugColor::eARGB_RED);
			else
				out << PxU32(PxDebugColor::eARGB_GREEN);
			out << DebugBox(getBounds3(mCacheBounds));
		}
	}
}

// This is the generic sweep test for all swept volumes, but not character-controller specific
// 这是所有扫描体积的通用扫描测试，但不是特定于character-controller的
bool SweepTest::doSweepTest(const InternalCBData_FindTouchedGeom* userData,
							InternalCBData_OnHit* userHitData,
							const UserObstacles& userObstacles,
							SweptVolume& swept_volume,
							const PxVec3& direction, const PxVec3& sideVector, PxU32 max_iter, PxU32* nb_collisions,
							float min_dist, const PxControllerFilters& filters, SweepPass sweepPass,
							const PxRigidActor*& touchedActorOut, const PxShape*& touchedShapeOut, PxU64 contextID)
{
	// Early exit when motion is zero. Since the motion is decomposed into several vectors
	// and this function is called for each of them, it actually happens quite often.
    // 当运动为零时提前退出。 由于运动被分解为多个向量并且为每个向量调用此函数，因此它实际上经常发生。
	if(direction.isZero())
		return false;

	PX_PROFILE_ZONE("CharacterController.doSweepTest", contextID);
	PX_UNUSED(contextID);

	bool hasMoved = false;
	mFlags &= ~(STF_VALIDATE_TRIANGLE_DOWN|STF_TOUCH_OTHER_CCT|STF_TOUCH_OBSTACLE);
	touchedShapeOut = NULL;
	touchedActorOut = NULL;
	mTouchedObstacleHandle	= INVALID_OBSTACLE_HANDLE;

	PxExtendedVec3 currentPosition = swept_volume.mCenter;
	PxExtendedVec3 targetOrientation = swept_volume.mCenter;
	targetOrientation += direction;

	PxU32 NbCollisions = 0;
	while(max_iter--)
	{
		mNbIterations++;
		// Compute current direction                计算当前方向
		PxVec3 currentDirection = targetOrientation - currentPosition;

		// Make sure the new TBV is still valid     确保新的 TBV 仍然有效
		{
			// Compute temporal bounding box. We could use a capsule or an OBB instead:
            // 计算时间边界框。 我们可以使用胶囊或 OBB 代替：
			// - the volume would be smaller        体积会更小
			// - but the query would be slower      但查询会更慢
			// Overall it's unclear whether it's worth it or not.   总体来说值不值得还不得而知。
			// TODO: optimize this part ?           优化这部分？
			PxExtendedBounds3 temporalBox;
			swept_volume.computeTemporalBox(*this, temporalBox, currentPosition, currentDirection);

			// Gather touched geoms                 合并碰到的图形
			updateTouchedGeoms(userData, userObstacles, temporalBox, filters, sideVector);
		}

		const float Length = currentDirection.magnitude();
		if(Length<=min_dist) //Use <= to handle the case where min_dist is zero.    可能会是0
			break;

		currentDirection /= Length;

		// From Quake2: "if velocity is against the original velocity, stop dead to avoid tiny occilations in sloping corners"
        // “如果速度与原始速度相反，则停止，以避免在倾斜的角落出现微小的振荡”
		if((currentDirection.dot(direction)) <= 0.0f)
			break;

		// From this point, we're going to update the position at least once
        // 从现在开始，我们将至少更新一次位置
		hasMoved = true;

		// Find closest collision   // 找到最近的碰撞
		SweptContact C;
		C.mDistance = Length + mUserParams.mContactOffset;

		if(!CollideGeoms(this, swept_volume, mGeomStream, currentPosition, currentDirection, C, !mUserParams.mOverlapRecovery))
		{
			// no collision found => move to desired position   // 未发现碰撞 => 移动到所需位置
			currentPosition = targetOrientation;
			break;
		}

		PX_ASSERT(C.mGeom);	// If we reach this point, we must have touched a geom
        // 如果走到了这点，那一定是碰到了一个geom

		if(mUserParams.mOverlapRecovery && C.mDistance==0.0f)
		{
/*			SweptContact C;
			C.mDistance = 10.0f;
			CollideGeoms(this, swept_volume, mGeomStream, currentPosition, -currentDirection, C, true);
			currentPosition -= currentDirection*C.mDistance;

			C.mDistance = 10.0f;
			CollideGeoms(this, swept_volume, mGeomStream, currentPosition, currentDirection, C, true);
			const float DynSkin = mUserParams.mContactOffset;
			if(C.mDistance>DynSkin)
				currentPosition += currentDirection*(C.mDistance-DynSkin);*/

			const PxVec3 mtd = computeMTD(this, swept_volume, mGeomStream, currentPosition, mUserParams.mContactOffset);

			NbCollisions++;

			if(nb_collisions)
				*nb_collisions = NbCollisions;

#ifdef DEBUG_MTD
			printf("MTD FIXUP: %f %f %f\n", mtd.x - swept_volume.mCenter.x, mtd.y - swept_volume.mCenter.y, mtd.z - swept_volume.mCenter.z);
#endif
			swept_volume.mCenter.x = PxExtended(mtd.x);
			swept_volume.mCenter.y = PxExtended(mtd.y);
			swept_volume.mCenter.z = PxExtended(mtd.z);
			return hasMoved;
//			currentPosition.x = mtd.x;
//			currentPosition.y = mtd.y;
//			currentPosition.z = mtd.z;
//			continue;
		}

		bool preventVerticalMotion = false;
		bool stopSliding = true;
		if(C.mGeom->mType==TouchedGeomType::eUSER_BOX || C.mGeom->mType==TouchedGeomType::eUSER_CAPSULE)
		{
			if(sweepPass!=SWEEP_PASS_SENSOR)
			{
				// We touched a user object, typically another CCT, but can also be a user-defined obstacle
                // 我们接触了一个用户对象，通常是另一个 CCT，但也可以是用户定义的障碍物
				// PT: TODO: technically lines marked with (*) shouldn't be here... revisit later
                // PT: TODO: 技术上标有 (*) 的行不应该在这里...以后再看

				const PxObstacle* touchedObstacle = NULL;	// (*)
				ObstacleHandle	touchedObstacleHandle = INVALID_OBSTACLE_HANDLE;
	//			if(mValidateCallback)
				{
					PxInternalCBData_OnHit* internalData = static_cast<PxInternalCBData_OnHit*>(userHitData);	// (*)
					internalData->touchedObstacle = NULL;											// (*)
					internalData->touchedObstacleHandle = INVALID_OBSTACLE_HANDLE;
					const PxU32 behaviorFlags = userHitCallback(userHitData, C, currentDirection, Length);
					stopSliding = (behaviorFlags & PxControllerBehaviorFlag::eCCT_SLIDE)==0;		// (*)
					touchedObstacle = internalData->touchedObstacle;								// (*)
					touchedObstacleHandle = internalData->touchedObstacleHandle;
				}
	//			printf("INTERNAL: %d\n", int(touchedObstacle));

				if(sweepPass==SWEEP_PASS_DOWN)
				{
					// (*)
					if(touchedObstacle)
					{
						mFlags |= STF_TOUCH_OBSTACLE;

						mTouchedObstacleHandle = touchedObstacleHandle;
						if(!gUseLocalSpace)
						{
							mTouchedPos = toVec3(touchedObstacle->mPos);
						}
						else
						{
							mTouchedPosObstacle_World = toVec3(C.mWorldPos);
							mTouchedPosObstacle_Local = worldToLocal(*touchedObstacle, C.mWorldPos);
						}
					}
					else
					{
						mFlags |= STF_TOUCH_OTHER_CCT;
					}
				}
			}
		}
		else
		{
			const PxShape* touchedShape = reinterpret_cast<const PxShape*>(C.mGeom->mTGUserData);
			PX_ASSERT(touchedShape);
			const PxRigidActor* touchedActor = C.mGeom->mActor;
			PX_ASSERT(touchedActor);

			// We touched a normal object   // 我们碰到了一个普通的物体
			if(sweepPass==SWEEP_PASS_DOWN)
			{
				mFlags &= ~(STF_TOUCH_OTHER_CCT|STF_TOUCH_OBSTACLE);

#ifdef USE_CONTACT_NORMAL_FOR_SLOPE_TEST
				mFlags |= STF_VALIDATE_TRIANGLE_DOWN;
				mContactNormalDownPass = C.mWorldNormal;
#else

				// Work out if the shape is attached to a static or dynamic actor.
				// The slope limit is currently only considered when walking on static actors.
				// It is ignored for shapes attached attached to dynamics and kinematics.
				// TODO:  1. should we treat stationary kinematics the same as statics.
				//		  2. should we treat all kinematics the same as statics.
				//		  3. should we treat no kinematics the same as statics.
                // 确定形状是附加到静态还是动态actor。
                // 当前仅在静态 actor 上行走时考虑坡度限制。
                // 对于附加到动力学和运动学的形状，它被忽略。
                // TODO:  1. 我们是否应该像对待静力学一样对待静态运动学。
                //        2. 我们是否应该将所有运动学视为静力学。
                //        3. 我们是否应该将运动学视为静力学。
				if((touchedActor->getConcreteType() == PxConcreteType::eRIGID_STATIC) && (C.mInternalIndex!=PX_INVALID_U32))
				{
					mFlags |= STF_VALIDATE_TRIANGLE_DOWN;
					const PxTriangle& touchedTri = mWorldTriangles.getTriangle(C.mInternalIndex);
					const PxVec3& upDirection = mUserParams.mUpDirection;
					const float dp0 = touchedTri.verts[0].dot(upDirection);
					const float dp1 = touchedTri.verts[1].dot(upDirection);
					const float dp2 = touchedTri.verts[2].dot(upDirection);
					float dpmin = dp0;
					dpmin = physx::intrinsics::selectMin(dpmin, dp1);
					dpmin = physx::intrinsics::selectMin(dpmin, dp2);
					float dpmax = dp0;
					dpmax = physx::intrinsics::selectMax(dpmax, dp1);
					dpmax = physx::intrinsics::selectMax(dpmax, dp2);

					PxExtendedVec3 cacheCenter;
					getCenter(mCacheBounds, cacheCenter);
					const float offset = upDirection.dot(toVec3(cacheCenter));
					mTouchedTriMin = dpmin + offset;
					mTouchedTriMax = dpmax + offset;

					touchedTri.normal(mContactNormalDownPass);
				}
#endif
				// Update touched shape in down pass    // 更新向下传递的触摸形状
				touchedShapeOut = const_cast<PxShape*>(touchedShape);
				touchedActorOut = touchedActor;
//				mTouchedPos = getShapeGlobalPose(*touchedShape).p;
				const PxTransform shapeTransform = getShapeGlobalPose(*touchedShape, *touchedActor);
				const PxVec3 worldPos = toVec3(C.mWorldPos);
				mTouchedPosShape_World = worldPos;
				mTouchedPosShape_Local = shapeTransform.transformInv(worldPos);
			}
			else if(sweepPass==SWEEP_PASS_SIDE || sweepPass==SWEEP_PASS_SENSOR)
			{
				if((touchedActor->getConcreteType() == PxConcreteType::eRIGID_STATIC) && (C.mInternalIndex!=PX_INVALID_U32))
				{
					mFlags |= STF_VALIDATE_TRIANGLE_SIDE;
					const PxTriangle& touchedTri = mWorldTriangles.getTriangle(C.mInternalIndex);
					touchedTri.normal(mContactNormalSidePass);
//					printf("%f | %f | %f\n", mContactNormalSidePass.x, mContactNormalSidePass.y, mContactNormalSidePass.z);
					if(mUserParams.mPreventVerticalSlidingAgainstCeiling && mContactNormalSidePass.dot(mUserParams.mUpDirection)<0.0f)
						preventVerticalMotion = true;
				}
			}

			if(sweepPass!=SWEEP_PASS_SENSOR)
//			if(mValidateCallback)
			{
				const PxU32 behaviorFlags = shapeHitCallback(userHitData, C, currentDirection, Length);
				stopSliding = (behaviorFlags & PxControllerBehaviorFlag::eCCT_SLIDE)==0;		// (*)
			}
		}

		if(sweepPass==SWEEP_PASS_DOWN && !stopSliding)
		{
			// Trying to solve the following problem:
			// - by default, the CCT "friction" is infinite, i.e. a CCT will not slide on a slope (this is by design)
			// - this produces bad results when a capsule CCT stands on top of another capsule CCT, without sliding. Visually it looks
			//   like the character is standing on the other character's head, it looks bad. So, here, we would like to let the CCT
			//   slide away, i.e. we don't want friction.
			// So here we simply increase the number of iterations (== let the CCT slide) when the first down collision is with another CCT.
            // 尝试解决以下问题：
            // - 默认情况下，CCT“摩擦”是无限的，即 CCT 不会在斜坡上滑动（这是设计使然）
            // - 当一个胶囊 CCT 站在另一个胶囊 CCT 的顶部而不滑动时，这会产生不好的结果。 
            //   从视觉上看，角色好像站在另一个角色的头上，看起来很糟糕。 所以，在这里，我们想让 CCT 滑开，即我们不想要摩擦。
            // 所以这里我们只是在第一次向下碰撞是与另一个 CCT 时增加迭代次数（== 让 CCT 滑动）。
			if(!NbCollisions)
				max_iter += 9;
//				max_iter += 1;
		}

		NbCollisions++;
//		mContactPointHeight = (float)C.mWorldPos[mUserParams.mUpDirection];	// UBI
		mContactPointHeight = toVec3(C.mWorldPos).dot(mUserParams.mUpDirection);	// UBI

		const float DynSkin = mUserParams.mContactOffset;

		if(C.mDistance>DynSkin/*+0.01f*/)
			currentPosition += currentDirection*(C.mDistance-DynSkin);
// DE6513
/*		else if(sweepPass==SWEEP_PASS_SIDE)
		{
			// Might be better to do a proper sweep pass here, in the opposite direction
			currentPosition += currentDirection*(C.mDistance-DynSkin);
		}*/
//~DE6513

		PxVec3 WorldNormal = C.mWorldNormal;
		if(preventVerticalMotion || ((mFlags & STF_WALK_EXPERIMENT) && (mUserParams.mNonWalkableMode!=PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING)))
		{
			// Make sure the auto-step doesn't bypass this !
			// PT: cancel out normal compo
            // 确保自动步骤不会绕过这个！
            // PT: 取消正常组合
//			WorldNormal[mUserParams.mUpDirection]=0.0f;
//			WorldNormal.normalize();
			PxVec3 normalCompo, tangentCompo;
			Ps::decomposeVector(normalCompo, tangentCompo, WorldNormal, mUserParams.mUpDirection);
			WorldNormal = tangentCompo;
			WorldNormal.normalize();
		}

		const float Bump = 0.0f;	// ### doesn't work when !=0 because of Quake2 hack!  // 由于 Quake2 hack，当 !=0 时不起作用！
		const float Friction = 1.0f;
		collisionResponse(targetOrientation, currentPosition, currentDirection, WorldNormal, Bump, Friction, (mFlags & STF_NORMALIZE_RESPONSE)!=0);
	}

	if(nb_collisions)
		*nb_collisions = NbCollisions;

	// Final box position that should be reflected in the graphics engine   // 应在图形引擎中反映的最终框位置
	swept_volume.mCenter = currentPosition;

	// If we didn't move, don't update the box position at all (keeping possible lazy-evaluated structures valid)
    // 如果我们没有移动，则根本不更新框位置（保持可能的惰性求值结构有效
	return hasMoved;
}

// ### have a return code to tell if we really moved or not  有一个返回码来判断我们是否真的移动了
// Using swept code & direct position update (no physics engine)    使用扫描代码和直接位置更新（无物理引擎）
// This function is the generic character controller logic, valid for all swept volumes 
// 此函数是通用字符控制器逻辑，对所有扫描体积有效
PxControllerCollisionFlags SweepTest::moveCharacter(
					const InternalCBData_FindTouchedGeom* userData,
					InternalCBData_OnHit* userHitData,
					SweptVolume& volume,
					const PxVec3& direction,
					const UserObstacles& userObstacles,
					float min_dist,
					const PxControllerFilters& filters,
					bool constrainedClimbingMode,
					bool standingOnMoving,
					const PxRigidActor*& touchedActor,
					const PxShape*& touchedShape,
					PxU64 contextID)
{
	PX_PROFILE_ZONE("CharacterController.moveCharacter", contextID);
	PX_UNUSED(contextID);

	bool standingOnMovingUp = standingOnMoving;

	mFlags &= ~STF_HIT_NON_WALKABLE;
	PxControllerCollisionFlags CollisionFlags = PxControllerCollisionFlags(0);
	const PxU32 maxIter = MAX_ITER;	// 1 for "collide and stop"
	const PxU32 maxIterSides = maxIter;
	const PxU32 maxIterDown = ((mFlags & STF_WALK_EXPERIMENT) && mUserParams.mNonWalkableMode==PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING) ? maxIter : 1;
//	const PxU32 maxIterDown = 1;

	// ### this causes the artificial gap on top of chars   这会导致字符顶部的人为间隙
	float stepOffset = mUserParams.mStepOffset;	// Default step offset can be cancelled in some cases.

	// Save initial height
	const PxVec3& upDirection = mUserParams.mUpDirection;
	const PxExtended originalHeight = volume.mCenter.dot(upDirection);
    const PxExtended originalBottomPoint = originalHeight - PxExtended(volume.mHalfHeight);	// UBI

	// TEST! Disable auto-step when flying. Not sure this is really useful. 
    // 测试！ 飞行时禁用自动步进。 不确定这是否真的有用。
//	if(direction[upDirection]>0.0f)
	const float dir_dot_up = direction.dot(upDirection);
//printf("%f\n", dir_dot_up);
	if(dir_dot_up>0.0f)
	{
		mFlags |= STF_IS_MOVING_UP;

		// PT: this makes it fail on a platform moving up when jumping
		// However if we don't do that a jump when moving up a slope doesn't work anymore!
		// Not doing this also creates jittering when a capsule CCT jumps against another capsule CCT
        // PT：这使得它在跳跃时向上移动的平台上失败
        // 但是，如果我们不这样做，向上移动时的跳跃将不再起作用！
        // 当一个胶囊 CCT 跳到另一个胶囊 CCT 时，不这样做也会产生抖动
		if(!standingOnMovingUp)	// PT: if we're standing on something moving up it's safer to do the up motion anyway, even though this won't work well before we add the flag in TA13542
            // PT：如果我们站在向上移动的物体上，无论如何做向上运动都是安全的，即使在我们在 TA13542 中添加标志之前这不会很好地工作
		{
//			static int count=0;	printf("Cancelling step offset... %d\n", count++);
			stepOffset = 0.0f;
		}
	}
	else
	{
		mFlags &= ~STF_IS_MOVING_UP;
	}

	// Decompose motion into 3 independent motions: up, side, down
	// - if the motion is purely down (gravity only), the up part is needed to fight accuracy issues. For example if the
	// character is already touching the geometry a bit, the down sweep test might have troubles. If we first move it above
	// the geometry, the problems disappear.
	// - if the motion is lateral (character moving forward under normal gravity) the decomposition provides the autostep feature
	// - if the motion is purely up, the down part can be skipped
    /** 将运动分解为 3 个独立的运动：向上、侧向、向下
     *  - 如果运动纯粹是向下（仅重力），则需要向上部分来解决准确性问题。 
     *    例如，如果角色已经稍微接触了几何体，则向下扫描测试可能会出现问题。 如果我们首先将其移动到几何体上方，问题就会消失。
     *  - 如果运动是横向的（角色在正常重力下向前移动），则分解提供自动步进功能。
     *  - 如果运动纯粹是向上的，可以跳过向下的部分。
     */ 
	PxVec3 UpVector(0.0f, 0.0f, 0.0f);
	PxVec3 DownVector(0.0f, 0.0f, 0.0f);

	PxVec3 normal_compo, tangent_compo;
	Ps::decomposeVector(normal_compo, tangent_compo, direction, upDirection);

//	if(direction[upDirection]<0.0f)
	if(dir_dot_up<=0.0f)
//		DownVector[upDirection] = direction[upDirection];
		DownVector = normal_compo;
	else
//		UpVector[upDirection] = direction[upDirection];
		UpVector = normal_compo;

//	PxVec3 SideVector = direction;
//	SideVector[upDirection] = 0.0f;
	PxVec3 SideVector = tangent_compo;

	// If the side motion is zero, i.e. if the character is not really moving, disable auto-step.
	// This is important to prevent the CCT from automatically climbing on small objects that move
	// against it. We should climb over those only if there's a valid side motion from the player.
    // 如果侧面运动为零，即如果角色没有真正移动，请禁用自动步进。
    // 这对于防止 CCT 自动爬上与其移动的小物体很重要。 只有当玩家有有效的侧面动作时，我们才应该越过那些。
	const bool sideVectorIsZero = !standingOnMovingUp && Ps::isAlmostZero(SideVector);	// We can't use PxVec3::isZero() safely with arbitrary up vectors
    // 我们不能安全地将 PxVec3::isZero() 与任意向上向量一起使用
	// #### however if we do this the up pass is disabled, with bad consequences when the CCT is on a dynamic object!!
	// ### this line makes it possible to push other CCTs by jumping on them
    // ### 但是如果我们这样做，向上传递将被禁用，当 CCT 在动态对象上时会产生不良后果！！
    // ### 这一行可以通过跳转到其他 CCT 来推动它们
//	const bool sideVectorIsZero = false;
//	printf("sideVectorIsZero: %d\n", sideVectorIsZero);

//	if(!SideVector.isZero())
	if(!sideVectorIsZero)
//		UpVector[upDirection] += stepOffset;
		UpVector += upDirection*stepOffset;
//	printf("stepOffset: %f\n", stepOffset);

	// ==========[ Initial volume query ]===========================

	// PT: the main difference between this initial query and subsequent ones is that we use the
	// full direction vector here, not the components along each axis. So there is a good chance
	// that this initial query will contain all the motion we need, and thus subsequent queries
	// will be skipped.
    // PT：这个初始查询和后续查询之间的主要区别在于我们在这里使用完整的方向向量，而不是沿每个轴的分量。 
    // 所以这个初始查询很有可能包含我们需要的所有动作，因此后续查询将被跳过。
	{
		PxExtendedBounds3 temporalBox;
		volume.computeTemporalBox(*this, temporalBox, volume.mCenter, direction);

		// Gather touched geoms 合并碰到的图形
		updateTouchedGeoms(userData, userObstacles, temporalBox, filters, SideVector);
	}

	// ==========[ UP PASS ]===========================

	mCachedTriIndexIndex = 0;
	const bool performUpPass = true;
	PxU32 NbCollisions=0;

	PxU32 maxIterUp;
	if(mUserParams.mPreventVerticalSlidingAgainstCeiling)
		maxIterUp = 1;
	else
		maxIterUp = Ps::isAlmostZero(SideVector) ? maxIter : 1;

	if(performUpPass)
	{
//		printf("%f | %f | %f\n", UpVector.x, UpVector.y, UpVector.z);

		// Prevent user callback for up motion. This up displacement is artificial, and only needed for auto-stepping.
		// If we call the user for this, we might eventually apply upward forces to objects resting on top of us, even
		// if we visually don't move. This produces weird-looking motions.
        // 防止用户回调向上运动。 这种向上位移是人为的，仅用于自动步进。
        // 如果我们为此调用用户，我们最终可能会对位于我们上方的物体施加向上的力，即使
        // 如果我们在视觉上不动。 这会产生看起来很奇怪的动作。
//		mValidateCallback = false;
		// PT: actually I think the previous comment is wrong. It's not only needed for auto-stepping: when the character
		// jumps there's a legit up motion and the lack of callback in that case could need some object can't be pushed
		// by the character's 'head' (for example). So I now think it's better to use the callback all the time, and
		// let users figure out what to do using the available state (like "isMovingUp", etc).
        // PT: 其实我觉得之前的评论是错误的。 它不仅需要自动步进：当角色跳跃时，有一个合法的向上运动，
        // 并且在这种情况下缺少回调可能需要一些不能被角色的“头部”推动的对象（例如）。 
        // 所以我现在认为最好一直使用回调，并让用户使用可用状态（如“isMovingUp”等）弄清楚该怎么做。
//		mValidateCallback = true;

		// In the walk-experiment we explicitly want to ban any up motions, to avoid characters climbing slopes they shouldn't climb.
		// So let's bypass the whole up pass.
        // 在步行实验中，我们明确希望禁止任何向上运动，以避免角色爬上他们不应该爬的斜坡。所以让我们绕过整个向上传递。
		if(!(mFlags & STF_WALK_EXPERIMENT))
		{
			// ### maxIter here seems to "solve" the V bug  // maxIter 这里似乎“解决”了 V 错误
			if(doSweepTest(userData, userHitData, userObstacles, volume, UpVector, SideVector, maxIterUp, &NbCollisions, min_dist, filters, SWEEP_PASS_UP, touchedActor, touchedShape, contextID))
			{
				if(NbCollisions)
				{
					CollisionFlags |= PxControllerCollisionFlag::eCOLLISION_UP;

					// Clamp step offset to make sure we don't undo more than what we did   
                    // 抓住step offset 以确保我们不会撤消比我们所做的更多
                    float Delta = float(volume.mCenter.dot(upDirection) - originalHeight);
                    if(Delta<stepOffset)
					{
						stepOffset=Delta;
					}
				}
			}
		}
	}

	// ==========[ SIDE PASS ]===========================

	mCachedTriIndexIndex = 1;
//	mValidateCallback = true;
	const bool PerformSidePass = true;

	mFlags &= ~STF_VALIDATE_TRIANGLE_SIDE;
	if(PerformSidePass)
	{
		NbCollisions=0;
		//printf("BS:%.2f %.2f %.2f NS=%d\n", volume.mCenter.x, volume.mCenter.y, volume.mCenter.z, mNbCachedStatic);
		if(doSweepTest(userData, userHitData, userObstacles, volume, SideVector, SideVector, maxIterSides, &NbCollisions, min_dist, filters, SWEEP_PASS_SIDE, touchedActor, touchedShape, contextID))
		{
			if(NbCollisions)
				CollisionFlags |= PxControllerCollisionFlag::eCOLLISION_SIDES;
		}
		//printf("AS:%.2f %.2f %.2f NS=%d\n", volume.mCenter.x, volume.mCenter.y, volume.mCenter.z, mNbCachedStatic);

		if(1 && constrainedClimbingMode && volume.getType()==SweptVolumeType::eCAPSULE && !(mFlags & STF_VALIDATE_TRIANGLE_SIDE))
		{
			const float capsuleRadius = static_cast<const SweptCapsule&>(volume).mRadius;

			const float sideM = SideVector.magnitude();
			if(sideM<capsuleRadius)
			{
				const PxVec3 sensor = SideVector.getNormalized() * capsuleRadius;

				mFlags &= ~STF_VALIDATE_TRIANGLE_SIDE;
				NbCollisions=0;
				//printf("BS:%.2f %.2f %.2f NS=%d\n", volume.mCenter.x, volume.mCenter.y, volume.mCenter.z, mNbCachedStatic);
				const PxExtendedVec3 saved = volume.mCenter;
				doSweepTest(userData, userHitData, userObstacles, volume, sensor, SideVector, 1, &NbCollisions, min_dist, filters, SWEEP_PASS_SENSOR, touchedActor, touchedShape, contextID);
				volume.mCenter = saved;
			}
		}
	}

	// ==========[ DOWN PASS ]===========================

	mCachedTriIndexIndex = 2;
	const bool PerformDownPass = true;

	if(PerformDownPass)
	{
		NbCollisions=0;

//		if(!SideVector.isZero())	// We disabled that before so we don't have to undo it in that case
                                    // 我们之前禁用了它，所以在这种情况下我们不必撤消它
		if(!sideVectorIsZero)		// We disabled that before so we don't have to undo it in that case
//			DownVector[upDirection] -= stepOffset;	// Undo our artificial up motion    撤消我们的人工向上运动
			DownVector -= upDirection*stepOffset;	// Undo our artificial up motion

		mFlags &= ~STF_VALIDATE_TRIANGLE_DOWN;
		touchedShape = NULL;
		touchedActor = NULL;
		mTouchedObstacleHandle	= INVALID_OBSTACLE_HANDLE;

		// min_dist actually makes a big difference :(      // min_dist 实际上有很大的不同
		// AAARRRGGH: if we get culled because of min_dist here, mValidateTriangle never becomes valid!
        // AAARRRGGH：如果我们因为这里的 min_dist 被剔除，mValidateTriangle 永远不会变得有效！
		if(doSweepTest(userData, userHitData, userObstacles, volume, DownVector, SideVector, maxIterDown, &NbCollisions, min_dist, filters, SWEEP_PASS_DOWN, touchedActor, touchedShape, contextID))
		{
			if(NbCollisions)
			{
				if(dir_dot_up<=0.0f)	// PT: fix attempt
					CollisionFlags |= PxControllerCollisionFlag::eCOLLISION_DOWN;

				if(mUserParams.mHandleSlope && !(mFlags & (STF_TOUCH_OTHER_CCT|STF_TOUCH_OBSTACLE)))	// PT: I think the following fix shouldn't be performed when mHandleSlope is false.
                    // PT：我认为当 mHandleSlope 为 false 时不应执行以下修复。
				{
					// PT: the following code is responsible for a weird capsule behaviour,
					// when colliding against a highly tesselated terrain:
					// - with a large direction vector, the capsule gets stuck against some part of the terrain
					// - with a slower direction vector (but in the same direction!) the capsule manages to move
					// I will keep that code nonetheless, since it seems to be useful for them.
                    // PT：以下代码负责在与高度细分的地形碰撞时出现奇怪的胶囊行为：
                    // - 使用大方向向量时，胶囊会卡在地形的某些部分
                    // - 使用较慢的方向向量（但方向相同！）胶囊设法移动
                    // 尽管如此，我还是会保留该代码，因为它似乎对他们有用。
//printf("%d\n", mFlags & STF_VALIDATE_TRIANGLE_SIDE);

					// constrainedClimbingMode
					if((mFlags & STF_VALIDATE_TRIANGLE_SIDE) && testSlope(mContactNormalSidePass, upDirection, mUserParams.mSlopeLimit))
					{
//printf("%d\n", mFlags & STF_VALIDATE_TRIANGLE_SIDE);
                        if(constrainedClimbingMode && PxExtended(mContactPointHeight) > originalBottomPoint + PxExtended(stepOffset))
						{
							mFlags |= STF_HIT_NON_WALKABLE;
							if(!(mFlags & STF_WALK_EXPERIMENT))
								return CollisionFlags;
	//						printf("Contrained\n");
						}
					}
					//~constrainedClimbingMode
				}
			}
		}
		//printf("AD:%.2f %.2f %.2f NS=%d\n", volume.mCenter.x, volume.mCenter.y, volume.mCenter.z, mNbCachedStatic);
//		printf("%d\n", mTouchOtherCCT);

		// TEST: do another down pass if we're on a non-walkable poly
		// ### kind of works but still not perfect
		// ### could it be because we zero the Y impulse later?
		// ### also check clamped response vectors
        // 测试：如果我们在不可行走的多边形上，再做一次向下传球
        // ### 类型的作品，但仍然不完美
        // ### 可能是因为我们稍后将 Y 脉冲归零吗？
        // ### 还检查钳位响应向量
//		if(mUserParams.mHandleSlope && mValidateTriangle && direction[upDirection]<0.0f)
//		if(mUserParams.mHandleSlope && !mTouchOtherCCT  && !mTouchObstacle && mValidateTriangle && dir_dot_up<0.0f)
		if(mUserParams.mHandleSlope && !(mFlags & (STF_TOUCH_OTHER_CCT|STF_TOUCH_OBSTACLE)) && (mFlags & STF_VALIDATE_TRIANGLE_DOWN) && dir_dot_up<=0.0f)
		{
			PxVec3 Normal;
		#ifdef USE_CONTACT_NORMAL_FOR_SLOPE_TEST
			Normal = mContactNormalDownPass;
		#else
			//mTouchedTriangle.normal(Normal);
			Normal = mContactNormalDownPass;
		#endif

            const float touchedTriHeight = float(PxExtended(mTouchedTriMax) - originalBottomPoint);

/*			if(touchedTriHeight>mUserParams.mStepOffset)
			{
				if(constrainedClimbingMode && mContactPointHeight > originalBottomPoint + stepOffset)
				{
					mFlags |= STF_HIT_NON_WALKABLE;
					if(!(mFlags & STF_WALK_EXPERIMENT))
						return CollisionFlags;
				}
			}*/

			if(touchedTriHeight>mUserParams.mStepOffset && testSlope(Normal, upDirection, mUserParams.mSlopeLimit))
			{
				mFlags |= STF_HIT_NON_WALKABLE;
				// Early exit if we're going to run this again anyway...
                // 如果我们无论如何要再次运行它，请提前退出......
				if(!(mFlags & STF_WALK_EXPERIMENT))
					return CollisionFlags;
		/*		CatchScene()->GetRenderer()->AddLine(mTouchedTriangle.mVerts[0], mTouched.mVerts[1], ARGB_YELLOW);
				CatchScene()->GetRenderer()->AddLine(mTouchedTriangle.mVerts[0], mTouched.mVerts[2], ARGB_YELLOW);
				CatchScene()->GetRenderer()->AddLine(mTouchedTriangle.mVerts[1], mTouched.mVerts[2], ARGB_YELLOW);
		*/

				// ==========[ WALK EXPERIMENT ]===========================

				mFlags |= STF_NORMALIZE_RESPONSE;

                const PxExtended tmp = volume.mCenter.dot(upDirection);
                float Delta = tmp > originalHeight ? float(tmp - originalHeight) : 0.0f;
                Delta += fabsf(direction.dot(upDirection));
                float Recover = Delta;

				NbCollisions=0;
                const float MD = Recover < min_dist ? Recover/float(maxIter) : min_dist;

				PxVec3 RecoverPoint(0,0,0);
				RecoverPoint = -upDirection*Recover;

				// PT: we pass "SWEEP_PASS_UP" for compatibility with previous code, but it's technically wrong (this is a 'down' pass)
                // PT：我们传递“SWEEP_PASS_UP”是为了与之前的代码兼容，但技术上是错误的（这是一个“向下”传递）
				if(doSweepTest(userData, userHitData, userObstacles, volume, RecoverPoint, SideVector, maxIter, &NbCollisions, MD, filters, SWEEP_PASS_UP, touchedActor, touchedShape, contextID))
				{
		//			if(NbCollisions)	CollisionFlags |= COLLISION_Y_DOWN;
					// PT: why did we do this ? Removed for now. It creates a bug (non registered event) when we land on a steep poly.
					// However this might have been needed when we were sliding on those polygons, and we didn't want the land anim to
					// start while we were sliding.
                    // PT：我们为什么要这样做？ 暂时删除了。 当我们降落在陡峭的多边形上时，它会产生一个错误（未注册的事件）。
                    // 但是，当我们在这些多边形上滑动时可能需要这样做，并且我们不希望在滑动时陆地动画开始。
		//			if(NbCollisions)	CollisionFlags &= ~PxControllerCollisionFlag::eCOLLISION_DOWN;
				}
				mFlags &= ~STF_NORMALIZE_RESPONSE;
			}
		}
	}

	return CollisionFlags;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This is an interface between NX users and the internal character controller module.
// 这是 NX 用户和内部角色控制器模块之间的接口。

#include "characterkinematic/PxControllerBehavior.h"
#include "PxActor.h"
#include "PxScene.h"

#include "CctInternalStructs.h"
#include "CctBoxController.h"
#include "CctCapsuleController.h"
#include "CctCharacterControllerManager.h"
#include "CctObstacleContext.h"

bool Controller::filterTouchedShape(const PxControllerFilters& filters)
{
	if(filters.mFilterCallback && (filters.mFilterFlags & PxQueryFlag::ePREFILTER))
	{
		const PxQueryFlags filterFlags = PxQueryFlag::eDYNAMIC|PxQueryFlag::ePREFILTER;
		const PxQueryFilterData filterData(filters.mFilterData ? *filters.mFilterData : PxFilterData(), filterFlags);
		PxHitFlags hitFlags = PxHitFlags(0);

		const PxQueryHitType::Enum retVal = filters.mFilterCallback->preFilter(filterData.data, mCctModule.mTouchedShape.get(), mCctModule.mTouchedActor.get(), hitFlags);
		if(retVal != PxQueryHitType::eNONE)
			return true;
		else
			return false;
	}

	return true;
}

void Controller::findTouchedObject(const PxControllerFilters& filters, const PxObstacleContext* obstacleContext, const PxVec3& upDirection)
{
	PX_ASSERT(!mCctModule.mTouchedShape && (mCctModule.mTouchedObstacleHandle == INVALID_OBSTACLE_HANDLE));

	// PT: the CCT works perfectly on statics without this extra mechanism, so we only raycasts against dynamics.
	// The pre-filter callback is used to filter out our own proxy actor shapes. We need to make sure our own filter
	// doesn't disturb the user-provided filter(s).

	// PT: for starter, if user doesn't want to collide against dynamics, we can skip the whole thing
	if(filters.mFilterFlags & PxQueryFlag::eDYNAMIC)
	{
		// PT: we use a local class instead of making "Controller" a PxQueryFilterCallback, since it would waste more memory.
		// Ideally we'd have a C-style callback and a user-data pointer, instead of being forced to create a class.
		class ControllerFilter : public PxQueryFilterCallback
		{
		public:
			PxQueryHitType::Enum	preFilter(const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags)
			{
				// PT: ignore triggers
				if(shape->getFlags() & physx::PxShapeFlag::eTRIGGER_SHAPE)
					return PxQueryHitType::eNONE;

				// PT: we want to discard our own internal shapes only
				if(mShapeHashSet->contains(const_cast<PxShape*>(shape)))
					return PxQueryHitType::eNONE;

				// PT: otherwise we revert to the user-callback, if it exists, and if users enabled that call
				if(mUserFilterCallback && (mUserFilterFlags & PxQueryFlag::ePREFILTER))
					return mUserFilterCallback->preFilter(filterData, shape, actor, queryFlags);

				return PxQueryHitType::eBLOCK;
			}

			PxQueryHitType::Enum	postFilter(const PxFilterData& filterData, const PxQueryHit& hit)
			{
				// PT: we may get called if users have asked for such a callback
				if(mUserFilterCallback && (mUserFilterFlags & PxQueryFlag::ePOSTFILTER))
					return mUserFilterCallback->postFilter(filterData, hit);

				PX_ASSERT(0);	// PT: otherwise we shouldn't have been called
				return PxQueryHitType::eNONE;
			}

			Ps::HashSet<PxShape*>*	mShapeHashSet;
			PxQueryFilterCallback*	mUserFilterCallback;
			PxQueryFlags			mUserFilterFlags;
		};

		ControllerFilter preFilter;
		preFilter.mShapeHashSet			= &mManager->mCCTShapes;
		preFilter.mUserFilterCallback	= filters.mFilterCallback;
		preFilter.mUserFilterFlags		= filters.mFilterFlags;

		// PT: for our own purpose we just want dynamics & pre-filter
		PxQueryFlags filterFlags = PxQueryFlag::eDYNAMIC|PxQueryFlag::ePREFILTER;
		// PT: but we may need the post-filter callback as well if users want it
		if(filters.mFilterFlags & PxQueryFlag::ePOSTFILTER)
			filterFlags |= PxQueryFlag::ePOSTFILTER;

		PxQueryFilterData filterData(filters.mFilterData ? *filters.mFilterData : PxFilterData(), filterFlags);

		const PxF32 probeLength = getHalfHeightInternal();	// Distance to feet
		const PxF32 extra = 0.0f;//probeLength * 0.1f;

		const PxVec3 rayOrigin = toVec3(mPosition);

		PxRaycastBuffer hit;
		hit.block.distance = FLT_MAX;
		if(mScene->raycast(rayOrigin, -upDirection, probeLength+extra, hit, PxHitFlags(0), filterData, &preFilter))
		{
			// copy touching hit to blocking so that the rest of the code works with .block
			hit.block = hit.getAnyHit(0);
			PX_ASSERT(hit.block.shape);
			PX_ASSERT(hit.block.actor);
			PX_ASSERT(hit.block.distance<=probeLength+extra);
			mCctModule.mTouchedShape = hit.block.shape;
			mCctModule.mTouchedActor = hit.block.actor;
//			mCctModule.mTouchedPos = getShapeGlobalPose(*hit.shape).p - upDirection*(probeLength-hit.distance);
			// PT: we only care about the up delta here
			const PxTransform shapeTransform = getShapeGlobalPose(*hit.block.shape, *hit.block.actor);
			mCctModule.mTouchedPosShape_World = PxVec3(0) - upDirection*(probeLength-hit.block.distance);
			mCctModule.mTouchedPosShape_Local = shapeTransform.transformInv(PxVec3(0));

			mPreviousSceneTimestamp = mScene->getTimestamp()-1;	// PT: just make sure cached timestamp is different
		}

		if(obstacleContext)
		{
			const ObstacleContext* obstacles = static_cast<const ObstacleContext*>(obstacleContext);
			PxRaycastHit obstacleHit;
			ObstacleHandle obstacleHandle;
			const PxObstacle* touchedObstacle = obstacles->raycastSingle(obstacleHit, rayOrigin, -upDirection, probeLength+extra, obstacleHandle);
//			printf("Touched raycast obstacle: %d\n", int(touchedObstacle));
			if(touchedObstacle && obstacleHit.distance<hit.block.distance)
			{
				PX_ASSERT(obstacleHit.distance<=probeLength+extra);
				mCctModule.mTouchedObstacleHandle = obstacleHandle;
				if(!gUseLocalSpace)
				{
					mCctModule.mTouchedPos = toVec3(touchedObstacle->mPos) - upDirection*(probeLength-obstacleHit.distance);
				}
				else
				{
					// PT: we only care about the up delta here
					mCctModule.mTouchedPosObstacle_World = PxVec3(0) - upDirection*(probeLength-obstacleHit.distance);
					mCctModule.mTouchedPosObstacle_Local = worldToLocal(*touchedObstacle, PxExtendedVec3(0,0,0));
				}
			}
		}
	}
}

bool Controller::rideOnTouchedObject(SweptVolume& volume, const PxVec3& upDirection, PxVec3& disp, const PxObstacleContext* obstacleContext)
{
	PX_ASSERT(mCctModule.mTouchedShape || (mCctModule.mTouchedObstacleHandle != INVALID_OBSTACLE_HANDLE));

	bool standingOnMoving = false;

	bool canDoUpdate = true;	// Always true on obstacles
	PxU32 behaviorFlags = 0;	// Default on shapes
	PxVec3 delta(0);
	float timeCoeff = 1.0f;

	if(mCctModule.mTouchedShape)
	{
		// PT: riding on a shape

		// PT: it is important to skip this stuff for static meshes,
		// otherwise accuracy issues create bugs like TA14007.
		const PxRigidActor& rigidActor = *mCctModule.mTouchedActor.get();
		if(rigidActor.getConcreteType()!=PxConcreteType::eRIGID_STATIC)
		{
			// PT: we only do the update when the timestamp has changed, otherwise "delta" will be zero
			// even if the underlying shape is moving.
			const PxU32 timestamp = mScene->getTimestamp();
//			printf("TimeStamp: %d\n", timestamp);
			canDoUpdate = timestamp!=mPreviousSceneTimestamp;
			if(canDoUpdate)
			{
				mPreviousSceneTimestamp = timestamp;

				timeCoeff = computeTimeCoeff();

				if(mBehaviorCallback)
					behaviorFlags = mBehaviorCallback->getBehaviorFlags(*mCctModule.mTouchedShape.get(), *mCctModule.mTouchedActor.get());

//				delta = getShapeGlobalPose(*mCctModule.mTouchedShape).p - mCctModule.mTouchedPos;
				const PxTransform shapeTransform = getShapeGlobalPose(*mCctModule.mTouchedShape.get(), rigidActor);
				const PxVec3 posPreviousFrame = mCctModule.mTouchedPosShape_World;
				const PxVec3 posCurrentFrame = shapeTransform.transform(mCctModule.mTouchedPosShape_Local);
				delta = posCurrentFrame - posPreviousFrame;
			}
		}
	}
	else
	{
		// PT: riding on an obstacle
		behaviorFlags = PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT;	// Default on obstacles

		timeCoeff = computeTimeCoeff();

		const PxObstacle* touchedObstacle = obstacleContext->getObstacleByHandle(mCctModule.mTouchedObstacleHandle);
		PX_ASSERT(touchedObstacle);

		if(mBehaviorCallback)
			behaviorFlags = mBehaviorCallback->getBehaviorFlags(*touchedObstacle);

		if(!gUseLocalSpace)
		{
			delta = toVec3(touchedObstacle->mPos) - mCctModule.mTouchedPos;
		}
		else
		{
			PxVec3 posPreviousFrame = mCctModule.mTouchedPosObstacle_World;
			PxVec3 posCurrentFrame = localToWorld(*touchedObstacle, mCctModule.mTouchedPosObstacle_Local);
			delta = posCurrentFrame - posPreviousFrame;
		}
	}

	if(canDoUpdate && !(behaviorFlags & PxControllerBehaviorFlag::eCCT_USER_DEFINED_RIDE))
	{
		// PT: amazingly enough even isAlmostZero doesn't solve this one.
		// Moving on a static mesh sometimes produces delta bigger than 1e-6f!
		// This may also explain the drift on some rotating platforms. It looks
		// like this delta computation is not very accurate.
//			standingOnMoving = !delta.isZero();
		standingOnMoving = !Ps::isAlmostZero(delta);
		mCachedStandingOnMoving = standingOnMoving;
//printf("%f %f %f\n", delta.x, delta.y, delta.z);
		if(standingOnMoving)
		{
			const float dir_dot_up = delta.dot(upDirection);
			const bool deltaMovingUp = dir_dot_up>0.0f;

			PxVec3 deltaUpDisp, deltaSideDisp;
			Ps::decomposeVector(deltaUpDisp, deltaSideDisp, delta, upDirection);

			if(deltaMovingUp)
			{
				volume.mCenter.x += PxExtended(deltaUpDisp.x);
				volume.mCenter.y += PxExtended(deltaUpDisp.y);
				volume.mCenter.z += PxExtended(deltaUpDisp.z);
			}
			else
			{
				disp += deltaUpDisp;
			}

			if(behaviorFlags & PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT)
				disp += deltaSideDisp;
		}
//		printf("delta in: %f %f %f (%f)\n", delta.x, delta.y, delta.z, 1.0f/timeCoeff);
		mDeltaXP = delta * timeCoeff;
	}
	else
	{
		standingOnMoving = mCachedStandingOnMoving;
	}
//	mDelta = delta;

	return standingOnMoving;
}

PxControllerCollisionFlags Controller::move(SweptVolume& volume, const PxVec3& originalDisp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacleContext, bool constrainedClimbingMode)
{
	const bool lockWrite = mManager->mLockingEnabled;   // 加写锁
	if(lockWrite)
		mWriteLock.lock();	

	mGlobalTime += PxF64(elapsedTime);                  // 总时间

	// Init CCT with per-controller settings            // 初始化CCT前置参数
	RenderBuffer* renderBuffer										= mManager->mRenderBuffer;
	const PxU32 debugRenderFlags									= mManager->mDebugRenderingFlags;
	mCctModule.mRenderBuffer										= renderBuffer;
	mCctModule.mRenderFlags											= debugRenderFlags;
	mCctModule.mUserParams											= mUserParams;
	mCctModule.mFlags												|= STF_FIRST_UPDATE;
	mCctModule.mUserParams.mMaxEdgeLength2							= mManager->mMaxEdgeLength * mManager->mMaxEdgeLength;
	mCctModule.mUserParams.mTessellation							= mManager->mTessellation;
	mCctModule.mUserParams.mOverlapRecovery							= mManager->mOverlapRecovery;
	mCctModule.mUserParams.mPreciseSweeps							= mManager->mPreciseSweeps;
	mCctModule.mUserParams.mPreventVerticalSlidingAgainstCeiling	= mManager->mPreventVerticalSlidingAgainstCeiling;
	mCctModule.resetStats();

	const PxVec3& upDirection = mUserParams.mUpDirection;

	///////////

	PxVec3 disp = originalDisp + mOverlapRecover;
	mOverlapRecover = PxVec3(0.0f);

    // CCT当前是否站在移动物体上
	bool standingOnMoving = false;	// PT: whether the CCT is currently standing on a moving object CCT 
	//printf("Touched shape: %d\n", int(mCctModule.mTouchedShape));
    //standingOnMoving=true;
    //printf("Touched obstacle: %d\n", int(mCctModule.mTouchedObstacle));

	if(mCctModule.mTouchedActor && mCctModule.mTouchedShape)
	{
		PxU32 nbShapes = mCctModule.mTouchedActor->getNbShapes();
		bool found = false;
		for(PxU32 i=0;i<nbShapes;i++)
		{
			PxShape* shape = NULL;
			mCctModule.mTouchedActor->getShapes(&shape, 1, i);
			if(mCctModule.mTouchedShape==shape)
			{
				found = true;
				break;
			}
		}

		if(!found)
		{
			mCctModule.mTouchedActor = NULL;
			mCctModule.mTouchedShape = NULL;
		}
		else
		{
			// check if we are still in the same scene  // 场景不一样
			if(mCctModule.mTouchedActor->getScene() != mScene)
			{
				mCctModule.mTouchedShape = NULL;
				mCctModule.mTouchedActor = NULL;
			}
			else
			{
				// check if the shape still does have the sq flag   // shape不是query_shape类型
				if(!(mCctModule.mTouchedShape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE))
				{
					mCctModule.mTouchedShape = NULL;
					mCctModule.mTouchedActor = NULL;
				}
				else
				{
					// invoke the CCT filtering for the shape       // 调用形状的 CCT 过滤
					if(!filterTouchedShape(filters))
					{
						mCctModule.mTouchedShape = NULL;
						mCctModule.mTouchedActor = NULL;
					}
				}
			}
		}
	}

    // 是否站在障碍物上
	if(!mCctModule.mTouchedShape && (mCctModule.mTouchedObstacleHandle == INVALID_OBSTACLE_HANDLE))
    {
        findTouchedObject(filters, obstacleContext, upDirection);
    }
		
	if(mCctModule.mTouchedShape || (mCctModule.mTouchedObstacleHandle != INVALID_OBSTACLE_HANDLE))
	{
        // 不在障碍物上，则检测一下是否在其它对象上
		standingOnMoving = rideOnTouchedObject(volume, upDirection, disp, obstacleContext);
	}
	else
	{
		mCachedStandingOnMoving = false;
		mDeltaXP = PxVec3(0.0f);
	}
//	printf("standingOnMoving: %d\n", standingOnMoving);

	///////////
	Ps::Array<const void*>&			boxUserData		= mManager->mBoxUserData;
	Ps::Array<PxExtendedBox>&		boxes			= mManager->mBoxes;
	Ps::Array<const void*>&			capsuleUserData	= mManager->mCapsuleUserData;
	Ps::Array<PxExtendedCapsule>&	capsules		= mManager->mCapsules;
	PX_ASSERT(!boxUserData.size());
	PX_ASSERT(!boxes.size());
	PX_ASSERT(!capsuleUserData.size());
	PX_ASSERT(!capsules.size());

	{
        // filter筛选controllers
		PX_PROFILE_ZONE("CharacterController.filterCandidateControllers", getContextId());

		// Experiment 实验 - to do better
		const PxU32 nbControllers = mManager->getNbControllers();
		Controller** controllers = mManager->getControllers();

		for(PxU32 i=0;i<nbControllers;i++)
		{
			Controller* currentController = controllers[i];
			if(currentController==this)
				continue;

			bool keepController = true;
			if(filters.mCCTFilterCallback)
				keepController = filters.mCCTFilterCallback->filter(*getPxController(), *currentController->getPxController());

			if(keepController)
			{
				if(currentController->mType==PxControllerShapeType::eBOX)
				{
					// PT: TODO: optimize 优化 this  
					BoxController* BC = static_cast<BoxController*>(currentController);
					PxExtendedBox obb;
					BC->getOBB(obb);

					boxes.pushBack(obb);

#ifdef REMOVED
					if(renderBuffer /*&& (debugRenderFlags & PxControllerDebugRenderFlag::eOBSTACLES)*/)
					{
						RenderOutput out(*renderBuffer);
						out << gCCTBoxDebugColor;

						out << PxTransform(toVec3(obb.center), obb.rot);

						out << DebugBox(obb.extents, true);
					}
#endif
					const size_t code = encodeUserObject(i, USER_OBJECT_CCT);
					boxUserData.pushBack(reinterpret_cast<const void*>(code));
				}
				else if(currentController->mType==PxControllerShapeType::eCAPSULE)
				{
					CapsuleController* CC = static_cast<CapsuleController*>(currentController);

					// PT: TODO: optimize this
					PxExtendedCapsule worldCapule;
					CC->getCapsule(worldCapule);
					capsules.pushBack(worldCapule);

					const size_t code = encodeUserObject(i, USER_OBJECT_CCT);
					capsuleUserData.pushBack(reinterpret_cast<const void*>(code));
				}
				else PX_ASSERT(0);
			}
		}
	}

	const ObstacleContext* obstacles = NULL;
	if(obstacleContext)
	{
		obstacles = static_cast<const ObstacleContext*>(obstacleContext);

		// PT: TODO: optimize this
		const PxU32 nbExtraBoxes = obstacles->mBoxObstacles.size();
		for(PxU32 i=0;i<nbExtraBoxes;i++)
		{
			const PxBoxObstacle& userBoxObstacle = obstacles->mBoxObstacles[i].mData;

			PxExtendedBox extraBox;
			extraBox.center		= userBoxObstacle.mPos;
			extraBox.extents	= userBoxObstacle.mHalfExtents;
			extraBox.rot		= userBoxObstacle.mRot;
			boxes.pushBack(extraBox);

			const size_t code = encodeUserObject(i, USER_OBJECT_BOX_OBSTACLE);
			boxUserData.pushBack(reinterpret_cast<const void*>(code));

			if(renderBuffer && (debugRenderFlags & PxControllerDebugRenderFlag::eOBSTACLES))
			{
				RenderOutput out(*renderBuffer);
				out << gObstacleDebugColor;

				out << PxTransform(toVec3(userBoxObstacle.mPos), userBoxObstacle.mRot);

				out << DebugBox(userBoxObstacle.mHalfExtents, true);
			}
		}

		const PxU32 nbExtraCapsules = obstacles->mCapsuleObstacles.size();
		for(PxU32 i=0;i<nbExtraCapsules;i++)
		{
			const PxCapsuleObstacle& userCapsuleObstacle = obstacles->mCapsuleObstacles[i].mData;

			PxExtendedCapsule extraCapsule;
			const PxVec3 capsuleAxis = userCapsuleObstacle.mRot.getBasisVector0() * userCapsuleObstacle.mHalfHeight;
			extraCapsule.p0		= PxExtendedVec3(	userCapsuleObstacle.mPos.x - PxExtended(capsuleAxis.x),
													userCapsuleObstacle.mPos.y - PxExtended(capsuleAxis.y),
													userCapsuleObstacle.mPos.z - PxExtended(capsuleAxis.z));
			extraCapsule.p1		= PxExtendedVec3(	userCapsuleObstacle.mPos.x + PxExtended(capsuleAxis.x),
													userCapsuleObstacle.mPos.y + PxExtended(capsuleAxis.y),
													userCapsuleObstacle.mPos.z + PxExtended(capsuleAxis.z));

			extraCapsule.radius	= userCapsuleObstacle.mRadius;
			capsules.pushBack(extraCapsule);
			const size_t code = encodeUserObject(i, USER_OBJECT_CAPSULE_OBSTACLE);
			capsuleUserData.pushBack(reinterpret_cast<const void*>(code));

			if(renderBuffer && (debugRenderFlags & PxControllerDebugRenderFlag::eOBSTACLES))
			{
				RenderOutput out(*renderBuffer);
				out << gObstacleDebugColor;
				out.outputCapsule(userCapsuleObstacle.mRadius, userCapsuleObstacle.mHalfHeight, PxTransform(toVec3(userCapsuleObstacle.mPos), userCapsuleObstacle.mRot));
			}
		}
	}


	UserObstacles userObstacles;

	const PxU32 nbBoxes = boxes.size();
	userObstacles.mNbBoxes			= nbBoxes;
	userObstacles.mBoxes			= nbBoxes ? boxes.begin() : NULL;
	userObstacles.mBoxUserData		= nbBoxes ? boxUserData.begin() : NULL;

	const PxU32 nbCapsules = capsules.size();
	userObstacles.mNbCapsules		= nbCapsules;
	userObstacles.mCapsules			= nbCapsules ? capsules.begin() : NULL;
	userObstacles.mCapsuleUserData	= nbCapsules ? capsuleUserData.begin() : NULL;

	PxInternalCBData_OnHit userHitData;
	userHitData.controller	= this;
	userHitData.obstacles	= obstacles;

	///////////

	PxControllerCollisionFlags collisionFlags = PxControllerCollisionFlags(0);

	PxInternalCBData_FindTouchedGeom findGeomData;
	findGeomData.scene				= mScene;
	findGeomData.renderBuffer		= renderBuffer;
	findGeomData.cctShapeHashSet	= &mManager->mCCTShapes;

	mCctModule.mFlags &= ~STF_WALK_EXPERIMENT;

	// store new touched actor/shape. Then set new actor/shape to avoid register/unregister for same objects
    // 存储新接触的actor/shape。 然后设置新的actor/shape以避免 register/unregister 相同的对象
	const PxRigidActor* touchedActor = NULL;
	const PxShape* touchedShape = NULL;
	PxExtendedVec3 Backup = volume.mCenter;
	collisionFlags = mCctModule.moveCharacter(&findGeomData, &userHitData, volume, disp, userObstacles, minDist, filters, constrainedClimbingMode, standingOnMoving, touchedActor, touchedShape, getContextId());

	if(mCctModule.mFlags & STF_HIT_NON_WALKABLE)
	{
		// A bit slow, but everything else I tried was less convincing...
        // 有点慢，但我尝试的其他办法更慢
		mCctModule.mFlags |= STF_WALK_EXPERIMENT;
		volume.mCenter = Backup;

		PxVec3 xpDisp;
		if(mUserParams.mNonWalkableMode==PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING)
		{
			PxVec3 tangent_compo;
			Ps::decomposeVector(xpDisp, tangent_compo, disp, upDirection);
		}
		else xpDisp = disp;

		collisionFlags = mCctModule.moveCharacter(&findGeomData, &userHitData, volume, xpDisp, userObstacles, minDist, filters, constrainedClimbingMode, standingOnMoving, touchedActor, touchedShape, getContextId());

		mCctModule.mFlags &= ~STF_WALK_EXPERIMENT;
	}
	mCctModule.mTouchedActor = touchedActor;
	mCctModule.mTouchedShape = touchedShape;

	mCollisionFlags = collisionFlags;

	// Copy results back
	mPosition = volume.mCenter;

	// Update kinematic actor
	if(mKineActor)
	{
		const PxVec3 delta = Backup - volume.mCenter;
		const PxF32 deltaM2 = delta.magnitudeSquared();
		if(deltaM2!=0.0f)
		{
			PxTransform targetPose = mKineActor->getGlobalPose();
			targetPose.p = toVec3(mPosition);
			targetPose.q = mUserParams.mQuatFromUp;
			mKineActor->setKinematicTarget(targetPose);
		}
	}

	mManager->resetObstaclesBuffers();

	if (lockWrite)
		mWriteLock.unlock();

	return collisionFlags;
}


PxControllerCollisionFlags BoxController::move(const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles)
{
	PX_PROFILE_ZONE("CharacterController.move", getContextId());

	PX_SIMD_GUARD;

	// Create internal swept box
	SweptBox sweptBox;
	sweptBox.mCenter		= mPosition;
	sweptBox.mExtents		= PxVec3(mHalfHeight, mHalfSideExtent, mHalfForwardExtent);
	sweptBox.mHalfHeight	= mHalfHeight;	// UBI
	return Controller::move(sweptBox, disp, minDist, elapsedTime, filters, obstacles, false);
}

PxControllerCollisionFlags CapsuleController::move(const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles)
{
	PX_PROFILE_ZONE("CharacterController.move", getContextId());

	PX_SIMD_GUARD;

	// Create internal swept capsule
	SweptCapsule sweptCapsule;
	sweptCapsule.mCenter		= mPosition;
	sweptCapsule.mRadius		= mRadius;
	sweptCapsule.mHeight		= mHeight;
	sweptCapsule.mHalfHeight	= mHeight*0.5f + mRadius;	// UBI
	return Controller::move(sweptCapsule, disp, minDist, elapsedTime, filters, obstacles, mClimbingMode==PxCapsuleClimbingMode::eCONSTRAINED);
}

