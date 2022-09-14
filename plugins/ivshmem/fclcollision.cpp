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

#include "fclcollision.hpp"

using boost::placeholders::_1;
using boost::placeholders::_2;

using namespace OpenRAVE;
using LinkConstPtr = OpenRAVE::KinBody::LinkConstPtr;

namespace fclrave {

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
        bodymanager.second->RemoveBody(body)
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
    CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
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
    CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
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
    CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
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

    CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
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

    CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
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
    CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);

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
    CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
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

} // namespace
