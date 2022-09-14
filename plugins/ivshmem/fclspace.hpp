// -*- coding: utf-8 -*-
#ifndef OPENRAVE_FCL_SPACE
#define OPENRAVE_FCL_SPACE

#include <boost/shared_ptr.hpp>
#include <functional>
#include <memory>
#include <vector>

#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/BVH/BVH_model.h>

#include <openrave/openrave.h>

namespace fclrave {

// Warning : this is the only place where we use std::shared_ptr (for compatibility with fcl)
typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
typedef std::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
typedef std::vector<fcl::CollisionObject *> CollisionGroup;
typedef std::pair<OpenRAVE::Transform, CollisionObjectPtr> TransformCollisionPair;

using MeshFactory = std::function<std::shared_ptr<fcl::CollisionGeometry>(const OpenRAVE::TriMesh&)>;

/// \brief fcl spaces manages the individual collision objects and sets up callbacks to track their changes.
///
/// It does not know or manage the broadphase manager
class FCLSpace
{
public:
    // corresponds to FCLUserData
    class FCLKinBodyInfo : public OpenRAVE::UserData
    {
    public:
        class FCLGeometryInfo
        {
        public:
            FCLGeometryInfo();

            FCLGeometryInfo(OpenRAVE::KinBody::GeometryPtr pgeom);

            ~FCLGeometryInfo();

            OpenRAVE::KinBody::GeometryPtr GetGeometry();

            boost::weak_ptr<OpenRAVE::KinBody::Geometry> _pgeom;
            std::string bodylinkgeomname; // for debugging purposes
            bool bFromKinBodyGeometry; ///< if true, then from kinbodygeometry. Otherwise from standalone object that does not have any KinBody associations
        };

        class LinkInfo
        {
        public:
            LinkInfo();

            LinkInfo(OpenRAVE::KinBody::LinkPtr plink);

            ~LinkInfo();

            void Reset();

            OpenRAVE::KinBody::LinkPtr GetLink();

            OpenRAVE::KinBody::LinkWeakPtr _plink;
            std::vector< boost::shared_ptr<FCLGeometryInfo> > vgeominfos; ///< info for every geometry of the link

            //int nLastStamp; ///< Tracks if the collision geometries are up to date wrt the body update stamp. This is for narrow phase collision
            TransformCollisionPair linkBV; ///< pair of the transformation and collision object corresponding to a bounding OBB for the link
            std::vector<TransformCollisionPair> vgeoms; ///< vector of transformations and collision object; one per geometries
            std::string bodylinkname; // for debugging purposes
            bool bFromKinBodyLink; ///< if true, then from kinbodylink. Otherwise from standalone object that does not have any KinBody associations
        };

        FCLKinBodyInfo();

        ~FCLKinBodyInfo();

        void Reset();

        OpenRAVE::KinBodyPtr GetBody();

        OpenRAVE::KinBodyWeakPtr _pbody;
        int nLastStamp;  ///< KinBody::GetUpdateStamp() when last synchronized ("is transform up to date")
        int nLinkUpdateStamp; ///< update stamp for link enable state (increases every time link enables change)
        int nGeometryUpdateStamp; ///< update stamp for geometry update state (increases every time geometry enables change)
        int nAttachedBodiesUpdateStamp; ///< update stamp for when attached bodies change of this body
        int nActiveDOFUpdateStamp; ///< update stamp for when active dofs change of this body

        std::vector< boost::shared_ptr<LinkInfo> > vlinks; ///< info for every link of the kinbody

        std::list<OpenRAVE::UserDataPtr> _linkEnabledCallbacks;

        std::string _geometrygroup; ///< name of the geometry group tracked by this kinbody info ; if empty, tracks the current geometries
    };

    using FCLKinBodyInfoPtr = boost::shared_ptr<FCLKinBodyInfo>;
    using LinkInfoPtr = boost::shared_ptr<FCLSpace::FCLKinBodyInfo::LinkInfo>;
    using SynchronizeCallbackFn = boost::function<void (FCLKinBodyInfoPtr)>;

    FCLSpace(OpenRAVE::EnvironmentBasePtr penv, const std::string& userdatakey);

    ~FCLSpace();

    void DestroyEnvironment();

    FCLKinBodyInfoPtr InitKinBody(OpenRAVE::KinBodyConstPtr pbody, FCLKinBodyInfoPtr pinfo = FCLKinBodyInfoPtr(), bool bSetToCurrentPInfo=true);

    bool HasNamedGeometry(const OpenRAVE::KinBody &body, const std::string& groupname);

    void SetGeometryGroup(const std::string& groupname);

    const std::string& GetGeometryGroup() const;

    bool SetBodyGeometryGroup(OpenRAVE::KinBodyConstPtr pbody, const std::string& groupname);

    const std::string& GetBodyGeometryGroup(const OpenRAVE::KinBody &body) const;

    // Set the current bvhRepresentation and reinitializes all the KinbodyInfo if needed
    void SetBVHRepresentation(std::string const &type);

    std::string const& GetBVHRepresentation() const;

    void Synchronize();

    void Synchronize(const OpenRAVE::KinBody &body);

    void SynchronizeWithAttached(const OpenRAVE::KinBody &body);

    FCLKinBodyInfoPtr& GetInfo(const OpenRAVE::KinBody &body);

    const FCLKinBodyInfoPtr& GetInfo(const OpenRAVE::KinBody &body) const;

    void RemoveUserData(OpenRAVE::KinBodyConstPtr pbody);

    /// \brief returns bodies initialized by this space. Note that some entries are null pointer.
    const std::vector<OpenRAVE::KinBodyConstPtr>& GetEnvBodies() const;

    inline CollisionObjectPtr GetLinkBV(const OpenRAVE::KinBody::Link &link);

    inline CollisionObjectPtr GetLinkBV(const OpenRAVE::KinBody &body, int index);

    inline CollisionObjectPtr GetLinkBV(const FCLKinBodyInfo &info, int index);


    inline LinkInfoPtr GetLinkInfo(const OpenRAVE::KinBody::Link &link);

    inline void SetIsSelfCollisionChecker(bool bIsSelfCollisionChecker);

    inline bool IsSelfCollisionChecker() const;

    inline const MeshFactory& GetMeshFactory() const;

    inline int GetEnvironmentId() const;
private:
    static void _AddGeomInfoToBVHSubmodel(fcl::BVHModel<fcl::OBB>& model, OpenRAVE::KinBody::GeometryInfo const &info);

    static TransformCollisionPair _CreateTransformCollisionPairFromOBB(fcl::OBB const &bv);

    // what about the tests on non-zero size (eg. box extents) ?
    std::shared_ptr<fcl::CollisionGeometry> _CreateFCLGeomFromGeometryInfo(const OpenRAVE::KinBody::GeometryInfo &info);

    /// \brief pass in info.GetBody() as a reference to avoid dereferencing the weak pointer in FCLKinBodyInfo
    void _Synchronize(FCLKinBodyInfo& info, const OpenRAVE::KinBody& body);

    void _ResetCurrentGeometryCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo);

    void _ResetGeometryGroupsCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo);

    void _ResetLinkEnableCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo);

    void _ResetActiveDOFsCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo);

    void _ResetAttachedBodyCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo);

    OpenRAVE::EnvironmentBasePtr _penv;
    std::string _userdatakey;
    std::string _geometrygroup;
    //SynchronizeCallbackFn _synccallback;

    std::string _bvhRepresentation;
    MeshFactory _meshFactory;

    std::vector<OpenRAVE::KinBodyConstPtr> _vecInitializedBodies; ///< vector of the kinbody initialized in this space. index is the environment body index. nullptr means uninitialized.
    std::vector<std::map< std::string, FCLKinBodyInfoPtr> > _cachedpinfo; ///< Associates to each body id and geometry group name the corresponding kinbody info if already initialized and not currently set as user data. Index of vector is the environment id. index 0 holds null pointer because kin bodies in the env should have positive index.
    std::vector<FCLKinBodyInfoPtr> _currentpinfo; ///< maps kinbody environment id to the kinbodyinfo struct constaining fcl objects. Index of the vector is the environment id (id of the body in the env, not __nUniqueId of env) of the kinbody at that index. The index being environment id makes it easier to compare objects without getting a handle to their pointers. Whenever a FCLKinBodyInfoPtr goes into this map, it is removed from _cachedpinfo. Index of vector is the environment id. index 0 holds null pointer because kin bodies in the env should have positive index.

    std::vector<int> _vecAttachedEnvBodyIndicesCache; ///< cache
    std::vector<OpenRAVE::KinBodyPtr> _vecAttachedBodiesCache; ///< cache

    bool _bIsSelfCollisionChecker; // Currently not used
};

}

#endif // OPENRAVE_FCL_SPACE
