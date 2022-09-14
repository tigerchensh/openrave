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

typedef FCLSpace::FCLKinBodyInfoPtr FCLKinBodyInfoPtr;
typedef FCLSpace::LinkInfoPtr LinkInfoPtr;

using BroadPhaseCollisionManagerPtr = std::shared_ptr<fcl::BroadPhaseCollisionManager>;

using GeometryPtr = OpenRAVE::KinBody::GeometryPtr;
using GeometryConstPtr = OpenRAVE::KinBody::GeometryConstPtr;

class FCLCollisionChecker : public OpenRAVE::CollisionCheckerBase
{
public:
    using OpenRAVE::InterfaceBase::shared_from_this;

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

    void Clone(OpenRAVE::InterfaceBasePtr preference, int cloningoptions);

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
    bool SetBroadphaseAlgorithmCommand(std::ostream& sout, std::istream& sinput)
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
    std::shared_ptr<fcl::BroadPhaseCollisionManager> _CreateManagerFromBroadphaseAlgorithm(std::string const &algorithm);

    /// Sets the bounding volume hierarchy representation which can be one of
    /// AABB, OBB, RSS, OBBRSS, kDOP16, kDOP18, kDOP24, kIOS
    /// e.g. "SetBVHRepresentation OBB"
    bool _SetBVHRepresentation(std::ostream& sout, std::istream& sinput)
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

    bool CheckCollision(const OpenRAVE::TriMesh& trimesh, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr());

    bool CheckCollision(const OpenRAVE::TriMesh& trimesh, CollisionReportPtr report = CollisionReportPtr());

    bool CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, CollisionReportPtr report = CollisionReportPtr());

    bool CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, const std::vector<OpenRAVE::KinBodyConstPtr>& vIncludedBodies, OpenRAVE::CollisionReportPtr report);

    bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckStandaloneSelfCollision(LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) override;

private:
    static bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data);

    bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb);

    static bool CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data);

    bool CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb);

    static bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist);

    bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb, fcl::FCL_REAL& dist);

    static bool CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist);

    bool CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb, fcl::FCL_REAL& dist);

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

    static auto MakeLinkPair(LinkConstPtr plink1, LinkConstPtr plink2) -> std::pair<LinkConstPtr, LinkConstPtr>
    {
        if( plink1.get() < plink2.get() ) {
            return std::make_pair(plink1, plink2);
        } else {
            return std::make_pair(plink2, plink1);
        }
    }

    std::pair<FCLSpace::FCLKinBodyInfo::LinkInfo*, LinkConstPtr> GetCollisionLink(const fcl::CollisionObject &collObj);

    std::pair<FCLSpace::FCLKinBodyInfo::FCLGeometryInfo*, GeometryConstPtr> GetCollisionGeometry(const fcl::CollisionObject &collObj);

    inline BroadPhaseCollisionManagerPtr _CreateManager() {
        return _CreateManagerFromBroadphaseAlgorithm(_broadPhaseCollisionManagerAlgorithm);
    }

    FCLCollisionManagerInstance& _GetBodyManager(KinBodyConstPtr pbody, bool bactiveDOFs);

    /// \brief gets environment manager corresponding to excludedBodyEnvIndices
    /// \param excludedBodyEnvIndices vector of environment body indices for excluded bodies. sorted in ascending order
    FCLCollisionManagerInstance& _GetEnvManager(const std::vector<int>& excludedBodyEnvIndices);

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
        for (const auto& body : _vCachedGrabbedBodies) {
            if (body->IsEnabled()) {
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

#ifdef NARROW_COLLISION_CACHING
    NarrowCollisionCache mCollisionCachedGuesses;
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
