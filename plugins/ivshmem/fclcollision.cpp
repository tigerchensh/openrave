// -*- coding: utf-8 -*-
// Copyright (C) 2022 Tan Li Boon (liboon.tan@mujin.co.jp)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <memory>

#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/shape/geometric_shapes.h>

#include "fclcollision.hpp"
#include "ravefcl.hpp"

using boost::placeholders::_1;
using boost::placeholders::_2;

using namespace OpenRAVE;
using LinkConstPtr = OpenRAVE::KinBody::LinkConstPtr;

namespace fclrave {

template<typename T>
inline bool IsIn(const T& x, const std::vector<T>&collection) {
    return std::find(collection.begin(), collection.end(), x) != collection.end();
}

FCLCollisionChecker::FCLCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
        : CollisionCheckerBase(penv)
        , _broadPhaseCollisionManagerAlgorithm("DynamicAABBTree2")
        , _bIsSelfCollisionChecker(true) // DynamicAABBTree2 should be slightly faster than Naive
{
    _bParentlessCollisionObject = false;
    _userdatakey = std::string("fclcollision") + boost::lexical_cast<std::string>(this);
    _fclspace.reset(new FCLSpace(penv, _userdatakey));
    _options = 0;
    // TODO : Should we put a more reasonable arbitrary value ?
    _numMaxContacts = std::numeric_limits<int>::max();
    _nGetEnvManagerCacheClearCount = 100000;
    __description = ":Interface Author: Kenji Maillard\n\nFlexible Collision Library collision checker";

    // TODO : Consider removing these which could be more harmful than anything else
    RegisterCommand("SetBroadphaseAlgorithm", boost::bind(&FCLCollisionChecker::SetBroadphaseAlgorithmCommand, this, _1, _2), "sets the broadphase algorithm (Naive, SaP, SSaP, IntervalTree, DynamicAABBTree, DynamicAABBTree_Array)");
    RegisterCommand("SetBVHRepresentation", boost::bind(&FCLCollisionChecker::_SetBVHRepresentation, this, _1, _2), "sets the Bouding Volume Hierarchy representation for meshes (AABB, OBB, OBBRSS, RSS, kIDS)");

    RAVELOG_VERBOSE_FORMAT("FCLCollisionChecker %s created in env %d", _userdatakey%penv->GetId());

    std::string broadphasealg, bvhrepresentation;
    sinput >> broadphasealg >> bvhrepresentation;
    if( broadphasealg != "" ) {
        _SetBroadphaseAlgorithm(broadphasealg);
    }
    if( bvhrepresentation != "" ) {
        _fclspace->SetBVHRepresentation(bvhrepresentation);
    }
}

FCLCollisionChecker::~FCLCollisionChecker() {
    RAVELOG_VERBOSE_FORMAT("FCLCollisionChecker %s destroyed in env %d", _userdatakey%GetEnv()->GetId());
    if (_maxNumBodyManagers > 0) {
        RAVELOG_DEBUG_FORMAT("env=%s FCLCollisionChecker=%s, number of body managers is current:%d, max:%d", GetEnv()->GetNameId()%_userdatakey%_bodymanagers.size()%_maxNumBodyManagers);
    }
    if (_maxNumEnvManagers > 0) {
        RAVELOG_DEBUG_FORMAT("env=%s FCLCollisionChecker=%s, number of env managers is current:%d, max:%d", GetEnv()->GetNameId()%_userdatakey%_envmanagers.size()%_maxNumEnvManagers);
    }

    DestroyEnvironment();
}

bool FCLCollisionChecker::SetCollisionOptions(int collisionoptions) {

}

int FCLCollisionChecker::GetCollisionOptions() const {

}

void FCLCollisionChecker::SetTolerance(dReal tolerance) {

}

bool FCLCollisionChecker::InitEnvironment() {
    RAVELOG_VERBOSE(str(boost::format("FCL User data initializing %s in env %d") % _userdatakey % GetEnv()->GetId()));
    _bIsSelfCollisionChecker = false;
    _fclspace->SetIsSelfCollisionChecker(false);
    std::vector<KinBodyPtr> vbodies;
    GetEnv()->GetBodies(vbodies);
    for (const auto& body : vbodies) {
        InitKinBody(body);
    }
    return true;
}

void FCLCollisionChecker::DestroyEnvironment() {
    RAVELOG_VERBOSE(str(boost::format("FCL User data destroying %s in env %d") % _userdatakey % GetEnv()->GetId()));
    _fclspace->DestroyEnvironment();
}

bool FCLCollisionChecker::InitKinBody(KinBodyPtr pbody) {
    OpenRAVE::EnvironmentLock lock(GetEnv()->GetMutex());
    FCLSpace::FCLKinBodyInfoPtr pinfo = _fclspace->GetInfo(*pbody);
    if( !pinfo || pinfo->GetBody() != pbody ) {
        pinfo = _fclspace->InitKinBody(pbody);
    }
    return !pinfo;
}

void FCLCollisionChecker::RemoveKinBody(KinBodyPtr pbody) {
    const OpenRAVE::KinBody& body = *pbody;

    // remove body from all the managers
    _bodymanagers.erase(std::make_pair(pbody.get(), (int)0));
    _bodymanagers.erase(std::make_pair(pbody.get(), (int)1));
    for (auto& bodymanager : _bodymanagers) {
        bodymanager.second->RemoveBody(body);
    }

    const int envBodyIndex = body.GetEnvironmentBodyIndex();
    EnvManagersMap::iterator it = _envmanagers.begin();
    int numErased = 0;
    while (it != _envmanagers.end()) {
        const std::vector<int>& excludedBodyIndices = it->first;
        auto itExcluded = lower_bound(excludedBodyIndices.begin(), excludedBodyIndices.end(), envBodyIndex);
        
        const bool bFound = itExcluded != excludedBodyIndices.end() && *itExcluded == envBodyIndex;
        if (bFound) {
            numErased++;
            it = _envmanagers.erase(it);
        }
        else {
            it->second->RemoveBody(body);
            ++it;
        }
    }
    if (numErased > 0) {
        RAVELOG_INFO_FORMAT("env=%s, erased %d element(s) from _envmanagers containing envBodyIndex=%d(\"%s\"), now %d remaining", GetEnv()->GetNameId()%numErased%envBodyIndex%body.GetName()%_envmanagers.size());
    }
    _fclspace->RemoveUserData(pbody);
}


bool FCLCollisionChecker::CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report) {
    // TODO : tailor this case when stuff become stable enough
    return CheckCollision(pbody1, std::vector<KinBodyConstPtr>(), std::vector<KinBody::LinkConstPtr>(), report);
}


bool FCLCollisionChecker::CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report) {
    if( !!report ) {
        report->Reset(_options);
    }

    if( pbody1->GetLinks().size() == 0 || !_IsEnabled(*pbody1) ) {
        return false;
    }

    if( pbody2->GetLinks().size() == 0 || !_IsEnabled(*pbody2) ) {
        return false;
    }

    if( pbody1->IsAttached(*pbody2) ) {
        return false;
    }

    _fclspace->SynchronizeWithAttached(*pbody1);
    _fclspace->SynchronizeWithAttached(*pbody2);

    // Do we really want to synchronize everything ?
    // We could put the synchronization directly inside GetBodyManager
    FCLCollisionManagerInstance& body1Manager = _GetBodyManager(pbody1, !!(_options & CO_ActiveDOFs));
    FCLCollisionManagerInstance& body2Manager = _GetBodyManager(pbody2, false); // TODO why are active DOFs not respected for pbody2??

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);
    if( _options & CO_Distance ) {
        if(!report) {
            throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
        }
        body1Manager.GetManager()->distance(body2Manager.GetManager().get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
    }
    body1Manager.GetManager()->collide(body2Manager.GetManager().get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
    return query._bCollision;
}


bool FCLCollisionChecker::CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) {
    // TODO : tailor this case when stuff become stable enough
    return CheckCollision(plink, std::vector<KinBodyConstPtr>(), std::vector<KinBody::LinkConstPtr>(), report);
}

bool FCLCollisionChecker::CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report) {
    if( !!report ) {
        report->Reset(_options);
    }

    if( !plink1->IsEnabled() || !plink2->IsEnabled() ) {
        return false;
    }

    KinBodyPtr plink1parent = plink1->GetParent(true);
    if( !plink1parent ) {
        throw OPENRAVE_EXCEPTION_FORMAT("Failed to get link %s parent", plink1parent->GetName(), OpenRAVE::ORE_InvalidArguments);
    }
    KinBodyPtr plink2parent = plink2->GetParent(true);
    if( !plink2parent ) {
        throw OPENRAVE_EXCEPTION_FORMAT("Failed to get link %s parent", plink2parent->GetName(), OpenRAVE::ORE_InvalidArguments);
    }

    _fclspace->SynchronizeWithAttached(*plink1parent);
    if( plink1parent != plink2parent ) {
        _fclspace->SynchronizeWithAttached(*plink2parent);
    }

    CollisionObjectPtr pcollLink1 = _fclspace->GetLinkBV(*plink1), pcollLink2 = _fclspace->GetLinkBV(*plink2);

    if( !pcollLink1 || !pcollLink2 ) {
        return false;
    }

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);
    if( _options & OpenRAVE::CO_Distance ) {
        if(!report) {
            throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
        }
        fcl::FCL_REAL dist = -1.0;
        CheckNarrowPhaseDistance(pcollLink1.get(), pcollLink2.get(), &query, dist);
    }
    if( !pcollLink1->getAABB().overlap(pcollLink2->getAABB()) ) {
        return false;
    }
    query.bselfCollision = true;  // for ignoring attached information!
    CheckNarrowPhaseCollision(pcollLink1.get(), pcollLink2.get(), &query);
    return query._bCollision;
}


bool FCLCollisionChecker::CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report) {
    if( !!report ) {
        report->Reset(_options);
    }

    if( !plink->IsEnabled() ) {
        return false;
    }

    if( pbody->GetLinks().size() == 0 || !_IsEnabled(*pbody) ) {
        return false;
    }

    if( pbody->IsAttached(*plink->GetParent()) ) {
        return false;
    }

    _fclspace->SynchronizeWithAttached(*plink->GetParent());
    _fclspace->SynchronizeWithAttached(*pbody);
    CollisionObjectPtr pcollLink = _fclspace->GetLinkBV(*plink);

    if( !pcollLink ) {
        return false;
    }

    FCLCollisionManagerInstance& bodyManager = _GetBodyManager(pbody, !!(_options & OpenRAVE::CO_ActiveDOFs)); // should also respect active dofs here

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);
    if( _options & OpenRAVE::CO_Distance ) {
        if(!report) {
            throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
        }
        bodyManager.GetManager()->distance(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
    }
    bodyManager.GetManager()->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
    return query._bCollision;
}

bool FCLCollisionChecker::CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report) {
    if( !!report ) {
        report->Reset(_options);
    }

    if( !plink->IsEnabled() || find(vlinkexcluded.begin(), vlinkexcluded.end(), plink) != vlinkexcluded.end() ) {
        return false;
    }

    _fclspace->Synchronize();
    CollisionObjectPtr pcollLink = _fclspace->GetLinkBV(*plink);

    if( !pcollLink ) {
        return false;
    }

    plink->GetParent()->GetAttachedEnvironmentBodyIndices(_attachedBodyIndicesCache);
    FCLCollisionManagerInstance& envManager = _GetEnvManager(_attachedBodyIndicesCache);

    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);
    if( _options & OpenRAVE::CO_Distance ) {
        if(!report) {
            throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
        }
        envManager.GetManager()->distance(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
    }
    envManager.GetManager()->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
    return query._bCollision;
}

bool FCLCollisionChecker::CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report) {\
    if( !!report ) {
        report->Reset(_options);
    }

    if( (pbody->GetLinks().size() == 0) || !_IsEnabled(*pbody) ) {
        return false;
    }

    _fclspace->Synchronize();
    FCLCollisionManagerInstance& bodyManager = _GetBodyManager(pbody, !!(_options & OpenRAVE::CO_ActiveDOFs));

    std::vector<int> attachedBodyIndices;
    pbody->GetAttachedEnvironmentBodyIndices(attachedBodyIndices);
    FCLCollisionManagerInstance& envManager = _GetEnvManager(attachedBodyIndices);

    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);
    if( _options & OpenRAVE::CO_Distance ) {
        if(!report) {
            throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
        }
        envManager.GetManager()->distance(bodyManager.GetManager().get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
    }
    envManager.GetManager()->collide(bodyManager.GetManager().get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

    return query._bCollision;
}

bool FCLCollisionChecker::CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report)  {
    RAVELOG_WARN("Ray collisions are not supported.\n");
    return false;
}

bool FCLCollisionChecker::CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report) {
    RAVELOG_WARN("Ray collisions are not supported.\n");
    return false;
}

bool FCLCollisionChecker::CheckCollision(const RAY& ray, CollisionReportPtr report) {
    RAVELOG_WARN("Ray collisions are not supported.\n");
    return false;
}

bool FCLCollisionChecker::CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)  {
    if( !!report ) {
        report->Reset(_options);
    }

    if( pbody->GetLinks().size() <= 1 ) {
        return false;
    }

    // We only want to consider the enabled links
    int adjacentOptions = KinBody::AO_Enabled;
    if( (_options & OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
        adjacentOptions |= KinBody::AO_ActiveDOFs;
    }

    const std::vector<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);
    // We need to synchronize after calling GetNonAdjacentLinks since it can move pbody even if it is const
    _fclspace->SynchronizeWithAttached(*pbody);

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);

    query.bselfCollision = true;
    FCLKinBodyInfoPtr pinfo = _fclspace->GetInfo(*pbody);
    for (const int set : nonadjacent) {
        size_t index1 = set&0xffff, index2 = set>>16;
        // We don't need to check if the links are enabled since we got adjacency information with AO_Enabled
        const FCLSpace::FCLKinBodyInfo::LinkInfo& pLINK1 = *pinfo->vlinks.at(index1);
        const FCLSpace::FCLKinBodyInfo::LinkInfo& pLINK2 = *pinfo->vlinks.at(index2);
        if( !pLINK1.linkBV.second || !pLINK2.linkBV.second || !pLINK1.linkBV.second->getAABB().overlap(pLINK2.linkBV.second->getAABB()) ) {
            continue;
        }
        for (auto& geom1 : pLINK1.vgeoms) {
            for (auto& geom2 : pLINK2.vgeoms) {
                if ( _options & OpenRAVE::CO_Distance ) {
                    if(!report) {
                        throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
                    }
                    fcl::FCL_REAL dist = -1.0;
                    CheckNarrowPhaseGeomDistance(geom1.second.get(), geom2.second.get(), &query, dist);
                }
                if( !geom1.second->getAABB().overlap(geom2.second->getAABB()) ) {
                    continue;
                }
                CheckNarrowPhaseGeomCollision(geom1.second.get(), geom2.second.get(), &query);
                if( !(_options & OpenRAVE::CO_Distance) && query._bStopChecking ) {
                    return query._bCollision;
                }
            }
        }
    }
    return query._bCollision;
}

bool FCLCollisionChecker::CheckStandaloneSelfCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report) {
    if( !!report ) {
        report->Reset(_options);
    }

    KinBodyPtr pbody = plink->GetParent();
    if( pbody->GetLinks().size() <= 1 ) {
        return false;
    }

    // We only want to consider the enabled links
    int adjacentOptions = KinBody::AO_Enabled;
    if( (_options & OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
        adjacentOptions |= KinBody::AO_ActiveDOFs;
    }

    const std::vector<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);
    // We need to synchronize after calling GetNonAdjacentLinks since it can move pbody env if it is const
    _fclspace->SynchronizeWithAttached(*pbody);

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);
    query.bselfCollision = true;
    FCLKinBodyInfoPtr pinfo = _fclspace->GetInfo(*pbody);

    for (const int set : nonadjacent) {
        int index1 = set&0xffff, index2 = set>>16;
        if( plink->GetIndex() == index1 || plink->GetIndex() == index2 ) {
            const FCLSpace::FCLKinBodyInfo::LinkInfo& pLINK1 = *pinfo->vlinks.at(index1);
            const FCLSpace::FCLKinBodyInfo::LinkInfo& pLINK2 = *pinfo->vlinks.at(index2);
            if( !pLINK1.linkBV.second || !pLINK2.linkBV.second || !pLINK1.linkBV.second->getAABB().overlap(pLINK2.linkBV.second->getAABB()) ) {
                continue;
            }
            for (auto& geom1 : pLINK1.vgeoms) {
                for (auto& geom2 : pLINK2.vgeoms) {
                    if ( _options & OpenRAVE::CO_Distance ) {
                        if(!report) {
                            throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
                        }
                        fcl::FCL_REAL dist = -1.0;
                        CheckNarrowPhaseGeomDistance(geom1.second.get(), geom2.second.get(), &query, dist);
                    }
                    if( !geom1.second->getAABB().overlap(geom2.second->getAABB()) ) {
                        continue;
                    }
                    CheckNarrowPhaseGeomCollision(geom1.second.get(), geom2.second.get(), &query);
                    if( !(_options & OpenRAVE::CO_Distance) && query._bStopChecking ) {
                        return query._bCollision;
                    }
                }
            }
        }
    }
    return query._bCollision;
}

//**
//** Non-Module Interface methods

bool FCLCollisionChecker::CheckCollision(const OpenRAVE::TriMesh& trimesh, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
{
    if( !!report ) {
        report->Reset(_options);
    }

    if( (pbody->GetLinks().size() == 0) || !_IsEnabled(*pbody) ) {
        return false;
    }

    _fclspace->SynchronizeWithAttached(*pbody);
    FCLCollisionManagerInstance& bodyManager = _GetBodyManager(pbody, !!(_options & OpenRAVE::CO_ActiveDOFs));

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);

    FCLSpace::FCLKinBodyInfo::LinkInfo objUserData;

    CollisionGeometryPtr ctrigeom = _fclspace->GetMeshFactory()(trimesh);
    ctrigeom->setUserData(nullptr);
    fcl::CollisionObject ctriobj(ctrigeom);
    //ctriobj.computeAABB(); // necessary?
    ctriobj.setUserData(&objUserData);
    bodyManager.GetManager()->collide(&ctriobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
    return query._bCollision;
}

bool FCLCollisionChecker::CheckCollision(const OpenRAVE::TriMesh& trimesh, CollisionReportPtr report)
{
    if( !!report ) {
        report->Reset(_options);
    }

    _fclspace->Synchronize();
    FCLCollisionManagerInstance& envManager = _GetEnvManager(std::vector<int>());

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);

    FCLSpace::FCLKinBodyInfo::LinkInfo objUserData;

    auto ctrigeom = _fclspace->GetMeshFactory()(trimesh);
    ctrigeom->setUserData(nullptr);
    fcl::CollisionObject ctriobj(ctrigeom);
    //ctriobj.computeAABB(); // necessary?
    ctriobj.setUserData(&objUserData);
    envManager.GetManager()->collide(&ctriobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
    return query._bCollision;
}

bool FCLCollisionChecker::CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, CollisionReportPtr report) 
{
    if( !!report ) {
        report->Reset(_options);
    }

    _fclspace->Synchronize();
    FCLCollisionManagerInstance& envManager = _GetEnvManager(std::vector<int>());

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);

    FCLSpace::FCLKinBodyInfo::LinkInfo objUserData;

    CollisionGeometryPtr cboxgeom = std::make_shared<fcl::Box>(ab.extents.x*2,ab.extents.y*2,ab.extents.z*2);
    cboxgeom->setUserData(nullptr);
    fcl::CollisionObject cboxobj(cboxgeom);

    fcl::Vec3f newPosition = ConvertVector(aabbPose * ab.pos);
    fcl::Quaternion3f newOrientation = ConvertQuaternion(aabbPose.rot);
    cboxobj.setTranslation(newPosition);
    cboxobj.setQuatRotation(newOrientation);
    cboxobj.computeAABB(); // probably necessary since translation changed
    cboxobj.setUserData(&objUserData);
    envManager.GetManager()->collide(&cboxobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
    return query._bCollision;
}

bool FCLCollisionChecker::CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, const std::vector<OpenRAVE::KinBodyConstPtr>& vIncludedBodies, OpenRAVE::CollisionReportPtr report) {
    if( !!report ) {
        report->Reset(_options);
    }

    _fclspace->Synchronize();
    std::vector<int> excludedBodyIndices;
    for (const KinBodyConstPtr& pbody : _fclspace->GetEnvBodies()) {
        if( !!pbody && find(vIncludedBodies.begin(), vIncludedBodies.end(), pbody) == vIncludedBodies.end() ) {
            const int envBodyIndex = pbody->GetEnvironmentBodyIndex();
            std::vector<int>::iterator it = lower_bound(excludedBodyIndices.begin(), excludedBodyIndices.end(), envBodyIndex);
            excludedBodyIndices.insert(it, envBodyIndex);
        }
    }
    FCLCollisionManagerInstance& envManager = _GetEnvManager(excludedBodyIndices);

    const std::vector<KinBodyConstPtr> vbodyexcluded;
    const std::vector<LinkConstPtr> vlinkexcluded;
    CollisionCallbackData query(shared_from_this(), report, vbodyexcluded, vlinkexcluded);

    FCLSpace::FCLKinBodyInfo::LinkInfo objUserData;

    CollisionGeometryPtr cboxgeom = std::make_shared<fcl::Box>(ab.extents.x*2,ab.extents.y*2,ab.extents.z*2);
    cboxgeom->setUserData(nullptr);
    fcl::CollisionObject cboxobj(cboxgeom);

    fcl::Vec3f newPosition = ConvertVector(aabbPose * ab.pos);
    fcl::Quaternion3f newOrientation = ConvertQuaternion(aabbPose.rot);
    cboxobj.setTranslation(newPosition);
    cboxobj.setQuatRotation(newOrientation);
    cboxobj.computeAABB(); // probably necessary since translation changed
    cboxobj.setUserData(&objUserData);
    envManager.GetManager()->collide(&cboxobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
    return query._bCollision;
}

// ** Narrowphase

bool FCLCollisionChecker::CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
    CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
    return pcb->_pchecker->CheckNarrowPhaseCollision(o1, o2, pcb);
}

bool FCLCollisionChecker::CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb)
{
    if( pcb->_bStopChecking ) {
        return true;     // don't test anymore
    }

    std::pair<FCLSpace::FCLKinBodyInfo::LinkInfo*, LinkConstPtr> o1info = GetCollisionLink(*o1), o2info = GetCollisionLink(*o2);

    if( !o1info.second ) {
        if( !o1info.first ) {
            if( _bParentlessCollisionObject ) {
                if( !!o2info.second ) {
                    RAVELOG_WARN_FORMAT("env=%s, fcl::CollisionObject o1 %x collides with link2 %s:%s, but collision ignored", GetEnv()->GetNameId()%o1%o2info.second->GetParent()->GetName()%o2info.second->GetName());
                }
            }
            return false;
        }
        // o1 is standalone object
    }
    if( !o2info.second ) {
        if( !o2info.first ) {
            if( _bParentlessCollisionObject ) {
                if( !!o1info.second ) {
                    RAVELOG_WARN_FORMAT("env=%s, link1 %s:%s collides with fcl::CollisionObject o2 %x, but collision ignored", GetEnv()->GetNameId()%o1info.second->GetParent()->GetName()%o1info.second->GetName()%o2);
                }
            }
            return false;
        }
        // o2 is standalone object
    }

    LinkConstPtr& plink1 = o1info.second;
    LinkConstPtr& plink2 = o2info.second;

    if( !!plink1 ) {
        if( !plink1->IsEnabled() ) {
            return false;
        }
        if( IsIn<KinBodyConstPtr>(plink1->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink1, pcb->_vlinkexcluded) ) {
            return false;
        }
    }

    if( !!plink2 ) {
        if( !plink2->IsEnabled() ) {
            return false;
        }
        if( IsIn<KinBodyConstPtr>(plink2->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink2, pcb->_vlinkexcluded) ) {
            return false;
        }
    }

    if( !!plink1 && !!plink2 ) {
        if( !pcb->bselfCollision && plink1->GetParent()->IsAttached(*plink2->GetParent())) {
            return false;
        }

        LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1), pLINK2 = _fclspace->GetLinkInfo(*plink2);

        for (auto& geompair1 : pLINK1->vgeoms) {
            for (auto& geompair2 : pLINK2->vgeoms) {
                if( geompair1.second->getAABB().overlap(geompair2.second->getAABB()) ) {
                    CheckNarrowPhaseGeomCollision(geompair1.second.get(), geompair2.second.get(), pcb);
                    if( pcb->_bStopChecking ) {
                        return true;
                    }
                }
            }
        }
    }
    else if( !!plink1 ) {
        LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1);
        for (auto& geompair1 : pLINK1->vgeoms) {
            if( geompair1.second->getAABB().overlap(o2->getAABB()) ) {
                CheckNarrowPhaseGeomCollision(geompair1.second.get(), o2, pcb);
                if( pcb->_bStopChecking ) {
                    return true;
                }
            }
        }
    }
    else if( !!plink2 ) {
        LinkInfoPtr pLINK2 = _fclspace->GetLinkInfo(*plink2);
        for (auto& geompair2 : pLINK2->vgeoms) {
            if( geompair2.second->getAABB().overlap(o1->getAABB()) ) {
                CheckNarrowPhaseGeomCollision(o1, geompair2.second.get(), pcb);
                if( pcb->_bStopChecking ) {
                    return true;
                }
            }
        }
    }

    if( pcb->_bCollision && !(_options & OpenRAVE::CO_AllLinkCollisions) ) {
        pcb->_bStopChecking = true; // stop checking collision
    }

    return pcb->_bStopChecking;
}


bool FCLCollisionChecker::CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
    CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
    return pcb->_pchecker->CheckNarrowPhaseGeomCollision(o1, o2, pcb);
}

bool FCLCollisionChecker::CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb)
{
    if( pcb->_bStopChecking ) {
        return true; // don't test anymore
    }

    pcb->_result.clear();

#ifdef NARROW_COLLISION_CACHING
    CollisionPair collpair = MakeCollisionPair(o1, o2);
    NarrowCollisionCache::iterator it = mCollisionCachedGuesses.find(collpair);
    if( it != mCollisionCachedGuesses.end() ) {
        pcb->_request.cached_gjk_guess = it->second;
    } else {
        // Is there anything more intelligent we could do there with the collision objects AABB ?
        pcb->_request.cached_gjk_guess = fcl::Vec3f(1,0,0);
    }
#endif

    size_t numContacts = fcl::collide(o1, o2, pcb->_request, pcb->_result);

#ifdef NARROW_COLLISION_CACHING
    mCollisionCachedGuesses[collpair] = pcb->_result.cached_gjk_guess;
#endif

    if( numContacts > 0 ) {
        if( !!pcb->_report ) {
            std::pair<FCLSpace::FCLKinBodyInfo::LinkInfo*, LinkConstPtr> o1info = GetCollisionLink(*o1), o2info = GetCollisionLink(*o2);
            auto o1geominfo = GetCollisionGeometry(*o1);
            auto o2geominfo = GetCollisionGeometry(*o2);

            LinkConstPtr& plink1 = o1info.second;
            LinkConstPtr& plink2 = o2info.second;
            GeometryPtr& pgeom1 = o1geominfo.second;
            GeometryPtr& pgeom2 = o2geominfo.second;

            // plink1 or plink2 can be None if object is standalone (ie coming from trimesh)

            //LinkConstPtr plink1 = GetCollisionLink(*o1), plink2 = GetCollisionLink(*o2);

            // these should be useless, just to make sure I haven't messed up
            //BOOST_ASSERT( plink1 && plink2 );
            //BOOST_ASSERT( plink1->IsEnabled() && plink2->IsEnabled() );
            if( !!plink1 && !!plink2 ) {
                BOOST_ASSERT( pcb->bselfCollision || !plink1->GetParent()->IsAttached(*plink2->GetParent()));
            }

            _reportcache.Reset(_options);
            _reportcache.plink1 = plink1;
            _reportcache.plink2 = plink2;
            _reportcache.pgeom1 = pgeom1;
            _reportcache.pgeom2 = pgeom2;

            // TODO : eliminate the contacts points (insertion sort (std::lower) + binary_search ?) duplicated
            // How comes that there are duplicated contacts points ?
            if( _options & (OpenRAVE::CO_Contacts | OpenRAVE::CO_AllGeometryContacts) ) {
                _reportcache.contacts.resize(numContacts);
                for(size_t i = 0; i < numContacts; ++i) {
                    fcl::Contact const &c = pcb->_result.getContact(i);
                    _reportcache.contacts[i] = CollisionReport::CONTACT(ConvertVector(c.pos), ConvertVector(c.normal), c.penetration_depth);
                }
            }


            if( pcb->_bHasCallbacks ) {
                OpenRAVE::CollisionAction action = OpenRAVE::CA_DefaultAction;
                CollisionReportPtr preport(&_reportcache, OpenRAVE::utils::null_deleter());
                for (auto& callback : pcb->GetCallbacks()) {
                    action = callback(preport, false);
                    if( action == OpenRAVE::CA_Ignore ) {
                        return false;
                    }
                }
            }

            pcb->_report->plink1 = _reportcache.plink1;
            pcb->_report->plink2 = _reportcache.plink2;
            pcb->_report->pgeom1 = _reportcache.pgeom1;
            pcb->_report->pgeom2 = _reportcache.pgeom2;
            if( pcb->_report->contacts.size() == 0) {
                pcb->_report->contacts.swap(_reportcache.contacts);
            } else {
                pcb->_report->contacts.reserve(pcb->_report->contacts.size() + numContacts);
                copy(_reportcache.contacts.begin(),_reportcache.contacts.end(), back_inserter(pcb->_report->contacts));
            }

            if( _options & OpenRAVE::CO_AllLinkCollisions ) {
                // We maintain vLinkColliding ordered
                auto linkPair = MakeLinkPair(plink1, plink2);
                auto end = pcb->_report->vLinkColliding.end();
                auto first = std::lower_bound(pcb->_report->vLinkColliding.begin(), end, linkPair);
                if( first == end || *first != linkPair ) {
                    pcb->_report->vLinkColliding.insert(first, linkPair);
                }
            }

            pcb->_bCollision = true;
            if( !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts)) ) {
                pcb->_bStopChecking = true; // stop checking collision
            }
            return pcb->_bStopChecking;
        }

        pcb->_bCollision = true;
        pcb->_bStopChecking = true; // since the report is NULL, there is no reason to continue
        return pcb->_bStopChecking;
    }

    return false; // keep checking collision
}

bool FCLCollisionChecker::CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist)
{
    CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
    return pcb->_pchecker->CheckNarrowPhaseDistance(o1, o2, pcb, dist);
}

bool FCLCollisionChecker::CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb, fcl::FCL_REAL& dist) {
    std::pair<FCLSpace::FCLKinBodyInfo::LinkInfo*, LinkConstPtr> o1info = GetCollisionLink(*o1), o2info = GetCollisionLink(*o2);

    if( !o1info.second && !o1info.first ) {
        // o1 is standalone object
        if( _bParentlessCollisionObject && !!o2info.second ) {
            RAVELOG_WARN_FORMAT("env=%s, fcl::CollisionObject o1 %x collides with link2 %s:%s, but is ignored for distance computation", GetEnv()->GetNameId()%o1%o2info.second->GetParent()->GetName()%o2info.second->GetName());
        }
        return false;
    }
    if( !o2info.second && !o2info.first ) {
        // o2 is standalone object
        if( _bParentlessCollisionObject && !!o1info.second ) {
            RAVELOG_WARN_FORMAT("env=%s, link1 %s:%s collides with fcl::CollisionObject o2 %x, but is ignored for distance computation", GetEnv()->GetNameId()%o1info.second->GetParent()->GetName()%o1info.second->GetName()%o2);
        }
        return false;
    }

    LinkConstPtr& plink1 = o1info.second;
    LinkConstPtr& plink2 = o2info.second;

    if( !!plink1 ) {
        if( !plink1->IsEnabled() ) {
            return false;
        }
        if( IsIn<KinBodyConstPtr>(plink1->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink1, pcb->_vlinkexcluded) ) {
            return false;
        }
    }

    if( !!plink2 ) {
        if( !plink2->IsEnabled() ) {
            return false;
        }
        if( IsIn<KinBodyConstPtr>(plink2->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink2, pcb->_vlinkexcluded) ) {
            return false;
        }
    }

    if( !!plink1 && !!plink2 ) {

        LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1), pLINK2 = _fclspace->GetLinkInfo(*plink2);

        //RAVELOG_VERBOSE_FORMAT("env=%d, link %s:%s with %s:%s", GetEnv()->GetId()%plink1->GetParent()->GetName()%plink1->GetName()%plink2->GetParent()->GetName()%plink2->GetName());
        for (auto& geompair1 : pLINK1->vgeoms) {
            for (auto& geompair2 : pLINK2->vgeoms) {
                CheckNarrowPhaseGeomDistance(geompair1.second.get(), geompair2.second.get(), pcb, dist);
            }
        }
    }
    else if( !!plink1 ) {
        LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1);
        for (auto& geompair1 : pLINK1->vgeoms) {
            CheckNarrowPhaseGeomDistance(geompair1.second.get(), o2, pcb, dist);
        }
    }
    else if( !!plink2 ) {
        LinkInfoPtr pLINK2 = _fclspace->GetLinkInfo(*plink2);
        for (auto& geompair2 : pLINK2->vgeoms) {
            CheckNarrowPhaseGeomDistance(o1, geompair2.second.get(), pcb, dist);
        }
    }

    return false;
}

bool FCLCollisionChecker::CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist) {
    CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
    return pcb->_pchecker->CheckNarrowPhaseGeomDistance(o1, o2, pcb, dist);
}


bool FCLCollisionChecker::CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb, fcl::FCL_REAL& dist) {
    // Compute the min distance between the objects.
    fcl::distance(o1, o2, pcb->_distanceRequest, pcb->_distanceResult);

    // If the min distance between these two objects is smaller than the min distance found so far, store it as the new min distance.
    if (pcb->_report->minDistance > pcb->_distanceResult.min_distance) {
        pcb->_report->minDistance = pcb->_distanceResult.min_distance;
    }

    // Store the current min distance.
    dist = pcb->_distanceResult.min_distance;

    // Can we ever stop without testing all objects?
    return false;
}

// ** Misc

void FCLCollisionChecker::Clone(InterfaceBasePtr preference, int cloningoptions) {
    CollisionCheckerBase::Clone(preference, cloningoptions);
    boost::shared_ptr<FCLCollisionChecker> r = boost::dynamic_pointer_cast<FCLCollisionChecker>(preference);
    // We don't clone Kinbody's specific geometry group
    _fclspace->SetGeometryGroup(r->GetGeometryGroup());
    _fclspace->SetBVHRepresentation(r->GetBVHRepresentation());
    _SetBroadphaseAlgorithm(r->GetBroadphaseAlgorithm());

    // We don't want to clone _bIsSelfCollisionChecker since a self collision checker can be created by cloning a environment collision checker
    _options = r->_options;
    _numMaxContacts = r->_numMaxContacts;
    RAVELOG_VERBOSE(str(boost::format("FCL User data cloning env %d into env %d") % r->GetEnv()->GetId() % GetEnv()->GetId()));
}


std::pair<FCLSpace::FCLKinBodyInfo::LinkInfo*, LinkConstPtr> FCLCollisionChecker::GetCollisionLink(const fcl::CollisionObject &collObj)
{
    FCLSpace::FCLKinBodyInfo::LinkInfo* link_raw = static_cast<FCLSpace::FCLKinBodyInfo::LinkInfo *>(collObj.getUserData());
    if( !!link_raw ) {
        const LinkConstPtr plink = link_raw->GetLink();
        if( !plink ) {
            if( link_raw->bFromKinBodyLink ) {
                RAVELOG_WARN_FORMAT("env=%s, The link %s was lost from fclspace (userdatakey %s)", GetEnv()->GetNameId()%link_raw->bodylinkname%_userdatakey);
            }
        }
        return std::make_pair(link_raw, plink);
    }
    RAVELOG_WARN_FORMAT("env=%s, fcl collision object %x does not have a link attached (userdatakey %s)", GetEnv()->GetNameId()%(&collObj)%_userdatakey);
    _bParentlessCollisionObject = true;
    return std::make_pair(link_raw, LinkConstPtr());
}

std::pair<FCLSpace::FCLKinBodyInfo::FCLGeometryInfo*, GeometryConstPtr> FCLCollisionChecker::GetCollisionGeometry(const fcl::CollisionObject &collObj)
{
    const std::shared_ptr<const fcl::CollisionGeometry>& collgeom = collObj.collisionGeometry();
    FCLSpace::FCLKinBodyInfo::FCLGeometryInfo* geom_raw = static_cast<FCLSpace::FCLKinBodyInfo::FCLGeometryInfo *>(collgeom->getUserData());
    if( !!geom_raw ) {
        const GeometryConstPtr pgeom = geom_raw->GetGeometry();
        if( !pgeom ) {
            if( geom_raw->bFromKinBodyGeometry ) {
                RAVELOG_WARN_FORMAT("env=%s, The geom %s was lost from fclspace (userdatakey %s)", GetEnv()->GetNameId()%geom_raw->bodylinkgeomname%_userdatakey);
            }
        }
        return std::make_pair(geom_raw, pgeom);
    }
    return std::make_pair(geom_raw, GeometryConstPtr());
}

std::shared_ptr<fcl::BroadPhaseCollisionManager> FCLCollisionChecker::_CreateManagerFromBroadphaseAlgorithm(std::string const &algorithm)
{
    if(algorithm == "Naive") {
        return std::make_shared<fcl::NaiveCollisionManager>();
    } else if(algorithm == "SaP") {
        return std::make_shared<fcl::SaPCollisionManager>();
    } else if(algorithm == "SSaP") {
        return std::make_shared<fcl::SSaPCollisionManager>();
    } else if(algorithm == "SpatialHashing") {
        throw OPENRAVE_EXCEPTION_FORMAT0("No spatial data provided, spatial hashing needs to be set up  with SetSpatialHashingBroadPhaseAlgorithm", OpenRAVE::ORE_InvalidArguments);
    } else if(algorithm == "IntervalTree") {
        return std::make_shared<fcl::IntervalTreeCollisionManager>();
    } else if(algorithm == "DynamicAABBTree") {
        return std::make_shared<fcl::DynamicAABBTreeCollisionManager>();
    } else if(algorithm == "DynamicAABBTree1") {
        auto pmanager = std::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        pmanager->tree_init_level = 1;
        return pmanager;
    } else if(algorithm == "DynamicAABBTree2") {
        auto pmanager = std::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        pmanager->tree_init_level = 2;
        return pmanager;
    } else if(algorithm == "DynamicAABBTree3") {
        auto pmanager = std::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        pmanager->tree_init_level = 3;
        return pmanager;
    } else if(algorithm == "DynamicAABBTree_Array") {
        return std::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
    } else if(algorithm == "DynamicAABBTree1_Array") {
        auto pmanager = std::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
        pmanager->tree_init_level = 1;
        return pmanager;
    } else if(algorithm == "DynamicAABBTree2_Array") {
        auto pmanager = std::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
        pmanager->tree_init_level = 2;
        return pmanager;
    } else if(algorithm == "DynamicAABBTree3_Array") {
        auto pmanager = std::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
        pmanager->tree_init_level = 3;
        return pmanager;
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Unknown broad-phase algorithm '%s'.", algorithm, OpenRAVE::ORE_InvalidArguments);
    }
}

FCLCollisionManagerInstance& FCLCollisionChecker::_GetBodyManager(KinBodyConstPtr pbody, bool bactiveDOFs)
{
    _bParentlessCollisionObject = false;
    BODYMANAGERSMAP::iterator it = _bodymanagers.find(std::make_pair(pbody.get(), (int)bactiveDOFs));
    if( it == _bodymanagers.end() ) {
        FCLCollisionManagerInstancePtr p(new FCLCollisionManagerInstance(*_fclspace, _CreateManager()));
        p->InitBodyManager(pbody, bactiveDOFs);
        it = _bodymanagers.insert(BODYMANAGERSMAP::value_type(std::make_pair(pbody.get(), (int)bactiveDOFs), p)).first;
        
        if ((int) _bodymanagers.size() > _maxNumBodyManagers) {
            RAVELOG_VERBOSE_FORMAT("env=%s, exceeded previous max number of body managers, now %d.", GetEnv()->GetNameId()%_bodymanagers.size());
            _maxNumBodyManagers = _bodymanagers.size();
        }
    }

    it->second->Synchronize();
    //RAVELOG_VERBOSE_FORMAT("env=%d, returning body manager cache %x (self=%d)", GetEnv()->GetId()%it->second.get()%_bIsSelfCollisionChecker);
    //it->second->PrintStatus(OpenRAVE::Level_Info);
    return *it->second;
}

FCLCollisionManagerInstance& FCLCollisionChecker::_GetEnvManager(const std::vector<int>& excludedBodyEnvIndices)
{
    _bParentlessCollisionObject = false;

    // check the cache and cleanup any unused environments
    // TODO come up with cleaner way of capping num of entries, maybe based on least-recently-used cache approach.
    if( --_nGetEnvManagerCacheClearCount < 0 ) {
        uint32_t curtime = OpenRAVE::utils::GetMilliTime();
        _nGetEnvManagerCacheClearCount = 100000;
        EnvManagersMap::iterator it = _envmanagers.begin();
        while(it != _envmanagers.end()) {
            if( (it->second->GetLastSyncTimeStamp() - curtime) > 10000 ) {
                //RAVELOG_VERBOSE_FORMAT("env=%d erasing manager at %u", GetEnv()->GetId()%it->second->GetLastSyncTimeStamp());
                _envmanagers.erase(it++);
            }
            else {
                ++it;
            }
        }
    }

    EnvManagersMap::iterator it = _envmanagers.find(excludedBodyEnvIndices);
    if( it == _envmanagers.end() ) {
        FCLCollisionManagerInstancePtr p(new FCLCollisionManagerInstance(*_fclspace, _CreateManager()));
        std::vector<int8_t> vecExcludedBodyEnvIndices(GetEnv()->GetMaxEnvironmentBodyIndex() + 1, 0);
        for (int excludeBodyIndex : excludedBodyEnvIndices) {
            vecExcludedBodyEnvIndices.at(excludeBodyIndex) = 1;
        }

        p->InitEnvironment(vecExcludedBodyEnvIndices);
        it = _envmanagers.insert(EnvManagersMap::value_type(excludedBodyEnvIndices, p)).first;

        if ((int) _envmanagers.size() > _maxNumEnvManagers) {
            RAVELOG_VERBOSE_FORMAT("env=%s, exceeded previous max number of env managers, now %d.", GetEnv()->GetNameId()%_envmanagers.size());
            _maxNumEnvManagers = _envmanagers.size();
        }
    }
    it->second->EnsureBodies(_fclspace->GetEnvBodies());
    it->second->Synchronize();
    //it->second->PrintStatus(OpenRAVE::Level_Info);
    //RAVELOG_VERBOSE_FORMAT("env=%d, returning env manager cache %x (self=%d)", GetEnv()->GetId()%it->second.get()%_bIsSelfCollisionChecker);
    return *it->second;
}

} // namespace
