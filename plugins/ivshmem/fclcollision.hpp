// -*- coding: utf-8 -*-
#ifndef OPENRAVE_FCL_COLLISION
#define OPENRAVE_FCL_COLLISION

#include <boost/unordered_set.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/utils.h>
#include <boost/function_output_iterator.hpp>

#include <openrave/openrave.h>

#include "fclspace.hpp"

#define FCLRAVE_CHECKPARENTLESS

using namespace OpenRAVE;

namespace fclrave {

#ifdef NARROW_COLLISION_CACHING
typedef std::pair<fcl::CollisionObject*, fcl::CollisionObject*> CollisionPair;

} // fclrave

namespace std {
template<>
struct hash<fclrave::CollisionPair>
{
    size_t operator()(const fclrave::CollisionPair& collpair) const {
        static const size_t shift = (size_t)log2(1 + sizeof(fcl::CollisionObject*));
        size_t seed = (size_t)(collpair.first) >> shift;
        boost::hash_combine(seed, (size_t)(collpair.second) >> shift);
        return seed;
    }

};
} // std

namespace fclrave {

typedef std::unordered_map<CollisionPair, fcl::Vec3f> NarrowCollisionCache;
#endif // NARROW_COLLISION_CACHING

typedef FCLSpace::FCLKinBodyInfoConstPtr FCLKinBodyInfoConstPtr;
typedef FCLSpace::FCLKinBodyInfoPtr FCLKinBodyInfoPtr;
typedef FCLSpace::LinkInfoPtr LinkInfoPtr;

template<typename T>
inline bool IsIn(T const& x, std::vector<T> const &collection) {
    return std::find(collection.begin(), collection.end(), x) != collection.end();
}

class FCLCollisionChecker : public OpenRAVE::CollisionCheckerBase
{
public:
    class CollisionCallbackData {
public:
        CollisionCallbackData(boost::shared_ptr<FCLCollisionChecker> pchecker, CollisionReportPtr report, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<LinkConstPtr>& vlinkexcluded) : _pchecker(pchecker), _report(report), _vbodyexcluded(vbodyexcluded), _vlinkexcluded(vlinkexcluded), bselfCollision(false), _bStopChecking(false), _bCollision(false)
        {
            _bHasCallbacks = _pchecker->GetEnv()->HasRegisteredCollisionCallbacks();
            if( _bHasCallbacks && !_report ) {
                _report.reset(new CollisionReport());
            }

            // TODO : What happens if we have CO_AllGeometryContacts set and not CO_Contacts ?
            // TODO not sure what's happening with FCL's contact computation. is it really disabled?
            if( !!report && !!(_pchecker->GetCollisionOptions() & OpenRAVE::CO_Contacts) ) {
                _request.num_max_contacts = _pchecker->GetNumMaxContacts();
                _request.enable_contact = true;
            } else {
                _request.enable_contact = false; // explicitly disable
            }

            // set the gjk solver (collision checking between convex bodies) so that we can use hints
            _request.gjk_solver_type = fcl::GST_INDEP;
            _distanceRequest.gjk_solver_type = fcl::GST_LIBCCD;

            if( !!_report ) {
                _report->Reset(_pchecker->GetCollisionOptions());
            }
        }

        const std::list<EnvironmentBase::CollisionCallbackFn>& GetCallbacks() {
            if( _bHasCallbacks &&( _listcallbacks.size() == 0) ) {
                _pchecker->GetEnv()->GetRegisteredCollisionCallbacks(_listcallbacks);
            }
            return _listcallbacks;
        }

        boost::shared_ptr<FCLCollisionChecker> _pchecker;
        fcl::CollisionRequest _request;
        fcl::CollisionResult _result;
        fcl::DistanceRequest _distanceRequest;
        fcl::DistanceResult _distanceResult;
        CollisionReportPtr _report;
        std::vector<KinBodyConstPtr> const& _vbodyexcluded;
        std::vector<LinkConstPtr> const& _vlinkexcluded;
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        bool bselfCollision;  ///< true if currently checking for self collision.
        bool _bStopChecking;  ///< if true, then stop the collision checking loop
        bool _bCollision;  ///< result of the collision

        bool _bHasCallbacks; ///< true if there's callbacks registered in the environment
        std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    };

    typedef boost::shared_ptr<CollisionCallbackData> CollisionCallbackDataPtr;

    FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);

    ~FCLCollisionChecker() override;

    void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);
        boost::shared_ptr<FCLCollisionChecker const> r = boost::dynamic_pointer_cast<FCLCollisionChecker const>(preference);
        // We don't clone Kinbody's specific geometry group
        _fclspace->SetGeometryGroup(r->GetGeometryGroup());
        _fclspace->SetBVHRepresentation(r->GetBVHRepresentation());
        _SetBroadphaseAlgorithm(r->GetBroadphaseAlgorithm());

        // We don't want to clone _bIsSelfCollisionChecker since a self collision checker can be created by cloning a environment collision checker
        _options = r->_options;
        _numMaxContacts = r->_numMaxContacts;
        RAVELOG_VERBOSE(str(boost::format("FCL User data cloning env %d into env %d") % r->GetEnv()->GetId() % GetEnv()->GetId()));
    }

    void SetNumMaxContacts(int numMaxContacts) {
        _numMaxContacts = numMaxContacts;
    }

    int GetNumMaxContacts() const {
        return _numMaxContacts;
    }

    void SetGeometryGroup(const std::string& groupname)
    {
        _fclspace->SetGeometryGroup(groupname);
    }

    const std::string& GetGeometryGroup() const
    {
        return _fclspace->GetGeometryGroup();
    }

    bool SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname)
    {
        return _fclspace->SetBodyGeometryGroup(pbody, groupname);
    }

    const std::string& GetBodyGeometryGroup(KinBodyConstPtr pbody) const
    {
        return _fclspace->GetBodyGeometryGroup(*pbody);
    }

    virtual bool SetCollisionOptions(int collision_options)
    {
        _options = collision_options;

        if( _options & OpenRAVE::CO_RayAnyHit ) {
            return false;
        }

        return true;
    }

    virtual int GetCollisionOptions() const
    {
        return _options;
    }

    virtual void SetTolerance(OpenRAVE::dReal tolerance)
    {
    }


    /// Sets the broadphase algorithm for collision checking
    /// The input algorithm can be one of : Naive, SaP, SSaP, IntervalTree, DynamicAABBTree{,1,2,3}, DynamicAABBTree_Array{,1,2,3}, SpatialHashing
    /// e.g. "SetBroadPhaseAlgorithm DynamicAABBTree"
    bool SetBroadphaseAlgorithmCommand(ostream& sout, istream& sinput)
    {
        std::string algorithm;
        sinput >> algorithm;
        _SetBroadphaseAlgorithm(algorithm);
        return !!sinput;
    }

    void _SetBroadphaseAlgorithm(const std::string &algorithm)
    {
        if(_broadPhaseCollisionManagerAlgorithm == algorithm) {
            return;
        }
        _broadPhaseCollisionManagerAlgorithm = algorithm;

        // clear all the current cached managers
        _bodymanagers.clear();
        _envmanagers.clear();
    }

    const std::string & GetBroadphaseAlgorithm() const {
        return _broadPhaseCollisionManagerAlgorithm;
    }

    // TODO : This is becoming really stupid, I should just add optional additional data for DynamicAABBTree
    BroadPhaseCollisionManagerPtr _CreateManagerFromBroadphaseAlgorithm(std::string const &algorithm)
    {
        if(algorithm == "Naive") {
            return boost::make_shared<fcl::NaiveCollisionManager>();
        } else if(algorithm == "SaP") {
            return boost::make_shared<fcl::SaPCollisionManager>();
        } else if(algorithm == "SSaP") {
            return boost::make_shared<fcl::SSaPCollisionManager>();
        } else if(algorithm == "SpatialHashing") {
            throw OPENRAVE_EXCEPTION_FORMAT0("No spatial data provided, spatial hashing needs to be set up  with SetSpatialHashingBroadPhaseAlgorithm", OpenRAVE::ORE_InvalidArguments);
        } else if(algorithm == "IntervalTree") {
            return boost::make_shared<fcl::IntervalTreeCollisionManager>();
        } else if(algorithm == "DynamicAABBTree") {
            return boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        } else if(algorithm == "DynamicAABBTree1") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
            pmanager->tree_init_level = 1;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree2") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
            pmanager->tree_init_level = 2;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree3") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
            pmanager->tree_init_level = 3;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree_Array") {
            return boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
        } else if(algorithm == "DynamicAABBTree1_Array") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager_Array> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
            pmanager->tree_init_level = 1;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree2_Array") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager_Array> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
            pmanager->tree_init_level = 2;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree3_Array") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager_Array> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
            pmanager->tree_init_level = 3;
            return pmanager;
        } else {
            throw OPENRAVE_EXCEPTION_FORMAT("Unknown broad-phase algorithm '%s'.", algorithm, OpenRAVE::ORE_InvalidArguments);
        }
    }

    /// Sets the bounding volume hierarchy representation which can be one of
    /// AABB, OBB, RSS, OBBRSS, kDOP16, kDOP18, kDOP24, kIOS
    /// e.g. "SetBVHRepresentation OBB"
    bool _SetBVHRepresentation(ostream& sout, istream& sinput)
    {
        std::string type;
        sinput >> type;
        _fclspace->SetBVHRepresentation(type);
        return !!sinput;
    }

    std::string const& GetBVHRepresentation() const {
        return _fclspace->GetBVHRepresentation();
    }


    bool InitEnvironment() override;

    void DestroyEnvironment() override;

    bool InitKinBody(OpenRAVE::KinBodyPtr pbody) override;

    void RemoveKinBody(OpenRAVE::KinBodyPtr pbody) override;

    bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, std::vector<OpenRAVE::KinBodyConstPtr> const &vbodyexcluded, std::vector<OpenRAVE::KinBody::LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(KinBodyConstPtr pbody, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<OpenRAVE::KinBody::LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(const OpenRAVE::RAY& ray, OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    bool CheckCollision(const OpenRAVE::RAY& ray, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    bool CheckCollision(const OpenRAVE::RAY& ray, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    virtual bool CheckCollision(const OpenRAVE::TriMesh& trimesh, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override
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
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);

        OPENRAVE_ASSERT_OP(trimesh.indices.size() % 3, ==, 0);
        size_t const num_points = trimesh.vertices.size();
        size_t const num_triangles = trimesh.indices.size() / 3;

        _fclPointsCache.resize(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
            Vector v = trimesh.vertices[ipoint];
            _fclPointsCache[ipoint] = fcl::Vec3f(v.x, v.y, v.z);
        }

        _fclTrianglesCache.resize(num_triangles);
        for (size_t itri = 0; itri < num_triangles; ++itri) {
            int const *const tri_indices = &trimesh.indices[3 * itri];
            _fclTrianglesCache[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        }

        FCLSpace::FCLKinBodyInfo::LinkInfo objUserData;

        CollisionGeometryPtr ctrigeom = _fclspace->GetMeshFactory()(_fclPointsCache, _fclTrianglesCache);
        ctrigeom->setUserData(nullptr);
        fcl::CollisionObject ctriobj(ctrigeom);
        //ctriobj.computeAABB(); // necessary?
        ctriobj.setUserData(&objUserData);
#ifdef FCLRAVE_CHECKPARENTLESS
        boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceB, this, boost::ref(*pbody), boost::ref(bodyManager)));
#endif
        bodyManager.GetManager()->collide(&ctriobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
    }

    virtual bool CheckCollision(const OpenRAVE::TriMesh& trimesh, CollisionReportPtr report = CollisionReportPtr()) override
    {
        if( !!report ) {
            report->Reset(_options);
        }

        _fclspace->Synchronize();
        FCLCollisionManagerInstance& envManager = _GetEnvManager(std::vector<int>());

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);

        OPENRAVE_ASSERT_OP(trimesh.indices.size() % 3, ==, 0);
        size_t const num_points = trimesh.vertices.size();
        size_t const num_triangles = trimesh.indices.size() / 3;

        _fclPointsCache.resize(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
            Vector v = trimesh.vertices[ipoint];
            _fclPointsCache[ipoint] = fcl::Vec3f(v.x, v.y, v.z);
        }

        _fclTrianglesCache.resize(num_triangles);
        for (size_t itri = 0; itri < num_triangles; ++itri) {
            int const *const tri_indices = &trimesh.indices[3 * itri];
            _fclTrianglesCache[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        }

        FCLSpace::FCLKinBodyInfo::LinkInfo objUserData;

        CollisionGeometryPtr ctrigeom = _fclspace->GetMeshFactory()(_fclPointsCache, _fclTrianglesCache);
        ctrigeom->setUserData(nullptr);
        fcl::CollisionObject ctriobj(ctrigeom);
        //ctriobj.computeAABB(); // necessary?
        ctriobj.setUserData(&objUserData);
#ifdef FCLRAVE_CHECKPARENTLESS
        //boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceB, this, boost::ref(*pbody), boost::ref(bodyManager)));
#endif
        envManager.GetManager()->collide(&ctriobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
    }

    virtual bool CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, CollisionReportPtr report = CollisionReportPtr()) override
    {
        if( !!report ) {
            report->Reset(_options);
        }

        _fclspace->Synchronize();
        FCLCollisionManagerInstance& envManager = _GetEnvManager(std::vector<int>());

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);

        FCLSpace::FCLKinBodyInfo::LinkInfo objUserData;

        CollisionGeometryPtr cboxgeom = make_shared<fcl::Box>(ab.extents.x*2,ab.extents.y*2,ab.extents.z*2);
        cboxgeom->setUserData(nullptr);
        fcl::CollisionObject cboxobj(cboxgeom);

        fcl::Vec3f newPosition = ConvertVectorToFCL(aabbPose * ab.pos);
        fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(aabbPose.rot);
        cboxobj.setTranslation(newPosition);
        cboxobj.setQuatRotation(newOrientation);
        cboxobj.computeAABB(); // probably necessary since translation changed
        cboxobj.setUserData(&objUserData);
#ifdef FCLRAVE_CHECKPARENTLESS
        //boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceB, this, boost::ref(*pbody), boost::ref(envManager)));
#endif
        envManager.GetManager()->collide(&cboxobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
    }

    virtual bool CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, const std::vector<OpenRAVE::KinBodyConstPtr>& vIncludedBodies, OpenRAVE::CollisionReportPtr report) override
    {
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
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);

        FCLSpace::FCLKinBodyInfo::LinkInfo objUserData;

        CollisionGeometryPtr cboxgeom = make_shared<fcl::Box>(ab.extents.x*2,ab.extents.y*2,ab.extents.z*2);
        cboxgeom->setUserData(nullptr);
        fcl::CollisionObject cboxobj(cboxgeom);

        fcl::Vec3f newPosition = ConvertVectorToFCL(aabbPose * ab.pos);
        fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(aabbPose.rot);
        cboxobj.setTranslation(newPosition);
        cboxobj.setQuatRotation(newOrientation);
        cboxobj.computeAABB(); // probably necessary since translation changed
        cboxobj.setUserData(&objUserData);
#ifdef FCLRAVE_CHECKPARENTLESS
        //boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceB, this, boost::ref(*pbody), boost::ref(envManager)));
#endif
        envManager.GetManager()->collide(&cboxobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
    }

    bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckStandaloneSelfCollision(LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) override;

private:
    inline boost::shared_ptr<FCLCollisionChecker> shared_checker() {
        return boost::static_pointer_cast<FCLCollisionChecker>(shared_from_this());
    }

    static bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseCollision(o1, o2, pcb);
    }

    bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb)
    {
        if( pcb->_bStopChecking ) {
            return true;     // don't test anymore
        }

//        _o1 = o1;
//        _o2 = o2;
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

            //RAVELOG_VERBOSE_FORMAT("env=%d, link %s:%s with %s:%s", GetEnv()->GetId()%plink1->GetParent()->GetName()%plink1->GetName()%plink2->GetParent()->GetName()%plink2->GetName());
            FOREACH(itgeompair1, pLINK1->vgeoms) {
                FOREACH(itgeompair2, pLINK2->vgeoms) {
                    if( itgeompair1->second->getAABB().overlap(itgeompair2->second->getAABB()) ) {
                        CheckNarrowPhaseGeomCollision(itgeompair1->second.get(), itgeompair2->second.get(), pcb);
                        if( pcb->_bStopChecking ) {
                            return true;
                        }
                    }
                }
            }
        }
        else if( !!plink1 ) {
            LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1);
            FOREACH(itgeompair1, pLINK1->vgeoms) {
                if( itgeompair1->second->getAABB().overlap(o2->getAABB()) ) {
                    CheckNarrowPhaseGeomCollision(itgeompair1->second.get(), o2, pcb);
                    if( pcb->_bStopChecking ) {
                        return true;
                    }
                }
            }
        }
        else if( !!plink2 ) {
            LinkInfoPtr pLINK2 = _fclspace->GetLinkInfo(*plink2);
            FOREACH(itgeompair2, pLINK2->vgeoms) {
                if( itgeompair2->second->getAABB().overlap(o1->getAABB()) ) {
                    CheckNarrowPhaseGeomCollision(o1, itgeompair2->second.get(), pcb);
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


    static bool CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseGeomCollision(o1, o2, pcb);
    }

    bool CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb)
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
                std::pair<FCLSpace::FCLKinBodyInfo::FCLGeometryInfo*, GeometryConstPtr> o1geominfo = GetCollisionGeometry(*o1), o2geominfo = GetCollisionGeometry(*o2);

                LinkConstPtr& plink1 = o1info.second;
                LinkConstPtr& plink2 = o2info.second;
                GeometryConstPtr& pgeom1 = o1geominfo.second;
                GeometryConstPtr& pgeom2 = o2geominfo.second;

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
                        _reportcache.contacts[i] = CollisionReport::CONTACT(ConvertVectorFromFCL(c.pos), ConvertVectorFromFCL(c.normal), c.penetration_depth);
                    }
                }


                if( pcb->_bHasCallbacks ) {
                    OpenRAVE::CollisionAction action = OpenRAVE::CA_DefaultAction;
                    CollisionReportPtr preport(&_reportcache, OpenRAVE::utils::null_deleter());
                    FOREACH(callback, pcb->GetCallbacks()) {
                        action = (*callback)(preport, false);
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
                    LinkPair linkPair = MakeLinkPair(plink1, plink2);
                    typedef std::vector< std::pair< LinkConstPtr, LinkConstPtr > >::iterator PairIterator;
                    PairIterator end = pcb->_report->vLinkColliding.end(), first = std::lower_bound(pcb->_report->vLinkColliding.begin(), end, linkPair);
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

    static bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist)
    {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseDistance(o1, o2, pcb, dist);
    }

    bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb, fcl::FCL_REAL& dist) {
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
            FOREACH(itgeompair1, pLINK1->vgeoms) {
                FOREACH(itgeompair2, pLINK2->vgeoms) {
                    CheckNarrowPhaseGeomDistance(itgeompair1->second.get(), itgeompair2->second.get(), pcb, dist);
                }
            }
        }
        else if( !!plink1 ) {
            LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1);
            FOREACH(itgeompair1, pLINK1->vgeoms) {
                CheckNarrowPhaseGeomDistance(itgeompair1->second.get(), o2, pcb, dist);
            }
        }
        else if( !!plink2 ) {
            LinkInfoPtr pLINK2 = _fclspace->GetLinkInfo(*plink2);
            FOREACH(itgeompair2, pLINK2->vgeoms) {
                CheckNarrowPhaseGeomDistance(o1, itgeompair2->second.get(), pcb, dist);
            }
        }

        return false;
    }

    static bool CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist) {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseGeomDistance(o1, o2, pcb, dist);
    }


    bool CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb, fcl::FCL_REAL& dist) {
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

#ifdef NARROW_COLLISION_CACHING
    static CollisionPair MakeCollisionPair(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
    {
        if( o1 < o2 ) {
            return make_pair(o1, o2);
        } else {
            return make_pair(o2, o1);
        }
    }
#endif

    static LinkPair MakeLinkPair(LinkConstPtr plink1, LinkConstPtr plink2)
    {
        if( plink1.get() < plink2.get() ) {
            return make_pair(plink1, plink2);
        } else {
            return make_pair(plink2, plink1);
        }
    }

    std::pair<FCLSpace::FCLKinBodyInfo::LinkInfo*, LinkConstPtr> GetCollisionLink(const fcl::CollisionObject &collObj)
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

    std::pair<FCLSpace::FCLKinBodyInfo::FCLGeometryInfo*, GeometryConstPtr> GetCollisionGeometry(const fcl::CollisionObject &collObj)
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

    inline BroadPhaseCollisionManagerPtr _CreateManager() {
        return _CreateManagerFromBroadphaseAlgorithm(_broadPhaseCollisionManagerAlgorithm);
    }

    FCLCollisionManagerInstance& _GetBodyManager(KinBodyConstPtr pbody, bool bactiveDOFs)
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

    /// \brief gets environment manager corresponding to excludedBodyEnvIndices
    /// \param excludedBodyEnvIndices vector of environment body indices for excluded bodies. sorted in ascending order
    FCLCollisionManagerInstance& _GetEnvManager(const std::vector<int>& excludedBodyEnvIndices)
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
            vector<int8_t> vecExcludedBodyEnvIndices(GetEnv()->GetMaxEnvironmentBodyIndex() + 1, 0);
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

    void _PrintCollisionManagerInstanceB(const KinBody& body, FCLCollisionManagerInstance& manager)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%s, self=%d, body %s ", GetEnv()->GetNameId()%_bIsSelfCollisionChecker%body.GetName());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstanceSelf(const KinBody& body)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%s, self=%d, body %s ", GetEnv()->GetNameId()%_bIsSelfCollisionChecker%body.GetName());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstanceBL(const KinBody& body, FCLCollisionManagerInstance& manager, const KinBody::Link& link)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%s, self=%d, body %s with link %s:%s (enabled=%d) ", GetEnv()->GetNameId()%_bIsSelfCollisionChecker%body.GetName()%link.GetParent()->GetName()%link.GetName()%link.IsEnabled());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstanceBE(const KinBody& body, FCLCollisionManagerInstance& manager, FCLCollisionManagerInstance& envManager)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%s, self=%d, body %s ", GetEnv()->GetNameId()%_bIsSelfCollisionChecker%body.GetName());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstance(const KinBody& body1, FCLCollisionManagerInstance& manager1, const KinBody& body2, FCLCollisionManagerInstance& manager2)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%s, self=%d, body1 %s (enabled=%d) body2 %s (enabled=%d) ", GetEnv()->GetNameId()%_bIsSelfCollisionChecker%body1.GetName()%body1.IsEnabled()%body2.GetName()%body2.IsEnabled());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstanceLE(const KinBody::Link& link, FCLCollisionManagerInstance& envManager)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%s, self=%d, link %s:%s (enabled=%d) ", GetEnv()->GetNameId()%_bIsSelfCollisionChecker%link.GetParent()->GetName()%link.GetName()%link.IsEnabled());
            _bParentlessCollisionObject = false;
        }
    }

    inline bool _IsEnabled(const KinBody& body)
    {
        if( body.IsEnabled() ) {
            return true;
        }

        // check if body has any enabled bodies
        body.GetGrabbed(_vCachedGrabbedBodies);
        FOREACH(itbody, _vCachedGrabbedBodies) {
            if( (*itbody)->IsEnabled() ) {
                return true;
            }
        }

        return false;
    }

    int _options;
    boost::shared_ptr<FCLSpace> _fclspace;
    int _numMaxContacts;
    std::string _userdatakey;
    std::string _broadPhaseCollisionManagerAlgorithm; ///< broadphase algorithm to use to create a manager. tested: Naive, DynamicAABBTree2

    typedef std::map< std::pair<const void*, int>, FCLCollisionManagerInstancePtr> BODYMANAGERSMAP; ///< Maps pairs of (body, bactiveDOFs) to oits manager
    BODYMANAGERSMAP _bodymanagers; ///< managers for each of the individual bodies. each manager should be called with InitBodyManager. Cannot use KinBodyPtr here since that will maintain a reference to the body!
    int _maxNumBodyManagers = 0; ///< for debug, record max size of _bodymanagers.

    typedef std::map<std::vector<int>, FCLCollisionManagerInstancePtr> EnvManagersMap; ///< Maps vector of excluded body indices to FCLCollisionManagerInstancePtr
    EnvManagersMap _envmanagers; // key is sorted vector of environment body indices of excluded bodies
    int _nGetEnvManagerCacheClearCount; ///< count down until cache can be cleared
    int _maxNumEnvManagers = 0; ///< for debug, record max size of _envmanagers.

#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
    std::map<fcl::CollisionObject*, int> _currentlyused;
    std::map<fcl::CollisionObject*, std::map<int, int> > _usestatistics;
#endif

#ifdef NARROW_COLLISION_CACHING
    NarrowCollisionCache mCollisionCachedGuesses;
#endif

#ifdef FCLUSESTATISTICS
    FCLStatisticsPtr _statistics;
#endif

    // In order to reduce allocations during collision checking

    CollisionReport _reportcache;
    std::vector<fcl::Vec3f> _fclPointsCache;
    std::vector<fcl::Triangle> _fclTrianglesCache;
    std::vector<KinBodyPtr> _vCachedGrabbedBodies;

    std::vector<int> _attachedBodyIndicesCache;

    bool _bIsSelfCollisionChecker; // Currently not used
    bool _bParentlessCollisionObject; ///< if set to true, the last collision command ran into colliding with an unknown object
};

} // fclrave

#endif
