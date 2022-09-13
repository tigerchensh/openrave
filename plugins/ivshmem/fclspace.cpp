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

#include <fcl/shape/geometric_shapes.h>

#include "fclspace.hpp"
#include "ravefcl.hpp"

using namespace OpenRAVE;

namespace fclrave {

FCLSpace::FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
    : _penv(penv), _userdatakey(userdatakey),
        _currentpinfo(1, FCLKinBodyInfoPtr()), // initialize with one null pointer, this is a place holder for null pointer so that we can return by reference. env id 0 means invalid so it's consistent with the definition as well
        _bIsSelfCollisionChecker(true)
{
    // After many test, OBB seems to be the only real option (followed by kIOS which is needed for distance checking)
    SetBVHRepresentation("OBB");
}

FCLSpace::~FCLSpace()
{
    DestroyEnvironment();
}

void FCLSpace::DestroyEnvironment()
{
    RAVELOG_VERBOSE_FORMAT("destroying fcl collision environment (env %d) (userdatakey %s)", _penv->GetId()%_userdatakey);
    for (KinBodyConstPtr pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        FCLKinBodyInfoPtr& pinfo = GetInfo(*pbody);
        if( !!pinfo ) {
            pinfo->Reset();
        }
    }
    // even after DestroyEnvironment is called, users of this class still try to access _currentpinfo
    // in that case, null pointer should be returned, instead of range error from _currentpinfo.at(0). for that purpose, keep the first element here.
    _currentpinfo.erase(_currentpinfo.begin() + 1, _currentpinfo.end());
    _cachedpinfo.clear();
    _vecInitializedBodies.clear();
}

FCLSpace::FCLKinBodyInfoPtr FCLSpace::InitKinBody(KinBodyConstPtr pbody, FCLKinBodyInfoPtr pinfo, bool bSetToCurrentPInfo)
{
    if( !pinfo ) {
        pinfo.reset(new FCLKinBodyInfo());
        pinfo->_geometrygroup = _geometrygroup;
    }

    RAVELOG_VERBOSE_FORMAT("env=%s, self=%d, init body %s (%d)", _penv->GetNameId()%_bIsSelfCollisionChecker%pbody->GetName()%pbody->GetEnvironmentBodyIndex());
    pinfo->Reset();
    pinfo->_pbody = boost::const_pointer_cast<KinBody>(pbody);
    // make sure that synchronization do occur !
    pinfo->nLastStamp = pbody->GetUpdateStamp() - 1;

    pinfo->vlinks.reserve(pbody->GetLinks().size());
    for (const auto& plink : pbody->GetLinks()) {
        boost::shared_ptr<FCLKinBodyInfo::LinkInfo> linkinfo(new FCLKinBodyInfo::LinkInfo(plink));

        fcl::AABB enclosingBV;

        // Glue code for a unified access to geometries
        if(pinfo->_geometrygroup.size() > 0 && plink->GetGroupNumGeometries(pinfo->_geometrygroup) >= 0) {
            const std::vector<KinBody::GeometryInfoPtr>& vgeometryinfos = plink->GetGeometriesFromGroup(pinfo->_geometrygroup);
            for (const auto& pgeominfo : vgeometryinfos) {
                const KinBody::GeometryInfo& geominfo = *pgeominfo;
                const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(geominfo);

                if( !pfclgeom ) {
                    continue;
                }
                pfclgeom->setUserData(nullptr);

                // We do not set the transformation here and leave it to _Synchronize
                CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                pfclcoll->setUserData(linkinfo.get());
                linkinfo->vgeoms.push_back(std::make_pair(geominfo.GetTransform(), pfclcoll));

                KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), geominfo);
                if( pgeominfo == vgeometryinfos.front() ) {
                    enclosingBV = ConvertAABB(_tmpgeometry.ComputeAABB(Transform()));
                }
                else {
                    enclosingBV += ConvertAABB(_tmpgeometry.ComputeAABB(Transform()));
                }
            }
        }
        else {
            const std::vector<KinBody::Link::GeometryPtr> & vgeometries = plink->GetGeometries();
            for (const auto& pgeom : vgeometries) {
                const KinBody::GeometryInfo& geominfo = pgeom->GetInfo();
                const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(geominfo);

                if( !pfclgeom ) {
                    continue;
                }
                boost::shared_ptr<FCLKinBodyInfo::FCLGeometryInfo> pfclgeominfo(new FCLKinBodyInfo::FCLGeometryInfo(pgeom));
                pfclgeominfo->bodylinkgeomname = pbody->GetName() + "/" + plink->GetName() + "/" + pgeom->GetName();
                pfclgeom->setUserData(pfclgeominfo.get());
                // save the pointers
                linkinfo->vgeominfos.push_back(pfclgeominfo);

                // We do not set the transformation here and leave it to _Synchronize
                CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                pfclcoll->setUserData(linkinfo.get());

                linkinfo->vgeoms.emplace_back(std::make_pair(geominfo.GetTransform(), pfclcoll));

                KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), geominfo);
                if( pgeom == vgeometries.front() ) {
                    enclosingBV = ConvertAABB(_tmpgeometry.ComputeAABB(Transform()));
                }
                else {
                    enclosingBV += ConvertAABB(_tmpgeometry.ComputeAABB(Transform()));
                }
            }
        }

        if( linkinfo->vgeoms.size() == 0 ) {
            RAVELOG_DEBUG_FORMAT("env=%s, Initializing body '%s' (index=%d) link '%s' with 0 geometries (env %d) (userdatakey %s)", _penv->GetNameId()%pbody->GetName()%pbody->GetEnvironmentBodyIndex()%plink->GetName()%_penv->GetId()%_userdatakey);
        }
        else {
            CollisionGeometryPtr pfclgeomBV = std::make_shared<fcl::Box>(enclosingBV.max_ - enclosingBV.min_);
            pfclgeomBV->setUserData(nullptr);
            CollisionObjectPtr pfclcollBV = boost::make_shared<fcl::CollisionObject>(pfclgeomBV);
            Transform trans(Vector(1,0,0,0), ConvertVector(0.5 * (enclosingBV.min_ + enclosingBV.max_)));
            pfclcollBV->setUserData(linkinfo.get());
            linkinfo->linkBV = std::make_pair(trans, pfclcollBV);
        }

        //link->nLastStamp = pinfo->nLastStamp;
        linkinfo->bodylinkname = pbody->GetName() + "/" + plink->GetName();
        pinfo->vlinks.push_back(linkinfo);
    }

    const int envId = pbody->GetEnvironmentBodyIndex();
    const int maxEnvId = _penv->GetMaxEnvironmentBodyIndex();
    BOOST_ASSERT(envId != 0);
    if( bSetToCurrentPInfo ) {
        _currentpinfo.at(envId) = pinfo;
    }
    //_cachedpinfo[pbody->GetEnvironmentBodyIndex()] what to do with the cache?
    _vecInitializedBodies.at(envId) = pbody;

    //Do I really need to synchronize anything at that point ?
    _Synchronize(*pinfo, *pbody);

    return pinfo;
}

bool FCLSpace::HasNamedGeometry(const KinBody &body, const std::string& groupname) {
    // The empty string corresponds to current geometries so all kinbodies have it
    if( groupname.size() == 0 ) {
        return true;
    }
    for (const auto& link : body.GetLinks()) {
        if( link->GetGroupNumGeometries(groupname) >= 0 ) {
            return true;
        }
    }
    return false;
}


void FCLSpace::SetGeometryGroup(const std::string& groupname)
{
    // should always do this since bodies can have different geometry groups set
    _geometrygroup = groupname;
    for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        SetBodyGeometryGroup(pbody, groupname);
    }
}

const std::string& FCLSpace::GetGeometryGroup() const
{
    return _geometrygroup;
}

bool FCLSpace::SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) {
    const KinBody& body = *pbody;

    if (!HasNamedGeometry(body, groupname)) {
        return false;
    }

    // Save the already existing FCLKinBodyInfoPtr for the old geometry group
    FCLKinBodyInfoPtr& poldinfo = GetInfo(body);
    if( poldinfo->_geometrygroup == groupname ) {
        return true;
    }

    poldinfo->nGeometryUpdateStamp += 1;

    const int maxBodyIndex = _penv->GetMaxEnvironmentBodyIndex();

    const int bodyIndex = body.GetEnvironmentBodyIndex();
    OPENRAVE_ASSERT_OP_FORMAT(bodyIndex, !=, 0, "env=%s, body %s", _penv->GetNameId()%body.GetName(), OpenRAVE::ORE_InvalidState);
    std::map< std::string, FCLKinBodyInfoPtr >& cache = _cachedpinfo.at(bodyIndex);
    cache[poldinfo->_geometrygroup] = poldinfo;

    FCLKinBodyInfoPtr& pinfo = cache[groupname];
    if(!pinfo) {
        RAVELOG_VERBOSE_FORMAT("FCLSpace : creating geometry %s for kinbody %s (id = %d) (env = %d)", groupname%body.GetName()%bodyIndex%_penv->GetId());
        pinfo.reset(new FCLKinBodyInfo);
        pinfo->_geometrygroup = groupname;
        InitKinBody(pbody, pinfo);
    }
    else {
        RAVELOG_VERBOSE_FORMAT("env=%s, switching to geometry %s for kinbody %s (id = %d)", _penv->GetNameId()%groupname%body.GetName()%bodyIndex);
        // Set the current info to use the FCLKinBodyInfoPtr associated to groupname
        _currentpinfo.at(bodyIndex) = pinfo;

        // Revoke the information inside the cache so that a potentially outdated object does not survive
        cache.erase(groupname);
    }

    return true;
}

const std::string& FCLSpace::GetBodyGeometryGroup(const KinBody &body) const {
    static const std::string empty;
    const FCLKinBodyInfoPtr& pinfo = GetInfo(body);
    if( !!pinfo ) {
        return pinfo->_geometrygroup;
    } else {
        return empty;
    }
}

void FCLSpace::SetBVHRepresentation(std::string const &type)
{
    if( type == _bvhRepresentation ) {
        return;
    }

    if (type == "AABB") {
        _bvhRepresentation = type;
        _meshFactory = &ConvertMeshToFCL<fcl::AABB>;
    } else if (type == "OBB") {
        _bvhRepresentation = type;
        _meshFactory = &ConvertMeshToFCL<fcl::OBB>;
    } else if (type == "RSS") {
        _bvhRepresentation = type;
        _meshFactory = &ConvertMeshToFCL<fcl::RSS>;
    } else if (type == "OBBRSS") {
        _bvhRepresentation = type;
        _meshFactory = &ConvertMeshToFCL<fcl::OBBRSS>;
    } else if (type == "kDOP16") {
        _bvhRepresentation = type;
        _meshFactory = &ConvertMeshToFCL< fcl::KDOP<16> >;
    } else if (type == "kDOP18") {
        _bvhRepresentation = type;
        _meshFactory = &ConvertMeshToFCL< fcl::KDOP<18> >;
    } else if (type == "kDOP24") {
        _bvhRepresentation = type;
        _meshFactory = &ConvertMeshToFCL< fcl::KDOP<24> >;
    } else if (type == "kIOS") {
        _bvhRepresentation = type;
        _meshFactory = &ConvertMeshToFCL<fcl::kIOS>;
    } else {
        RAVELOG_WARN(str(boost::format("Unknown BVH representation '%s', keeping '%s' representation") % type % _bvhRepresentation));
        return;
    }

    // reinitialize all the FCLKinBodyInfo

    for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        const KinBody& body = *pbody;
        FCLKinBodyInfoPtr& pinfo = GetInfo(body);
        pinfo->nGeometryUpdateStamp++;
        InitKinBody(pbody, pinfo);
    }
    _cachedpinfo.clear();
}

std::string const& FCLSpace::GetBVHRepresentation() const {
    return _bvhRepresentation;
}

void FCLSpace::Synchronize()
{
    // We synchronize only the initialized bodies, which differs from oderave
    for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        Synchronize(*pbody);
    }
}

void FCLSpace::Synchronize(const KinBody &body)
{
    FCLKinBodyInfoPtr& pinfo = GetInfo(body);
    if( !pinfo ) {
        return;
    }
    // expensive, comment out for now
    //BOOST_ASSERT( pinfo->_pbody.lock().get() == &body);
    _Synchronize(*pinfo, body);
}

void FCLSpace::SynchronizeWithAttached(const KinBody &body)
{
    if( body.HasAttached() ) {
        std::vector<int>& vecAttachedEnvBodyIndices = _vecAttachedEnvBodyIndicesCache;
        body.GetAttachedEnvironmentBodyIndices(vecAttachedEnvBodyIndices);
        std::vector<KinBodyPtr>& attachedBodies = _vecAttachedBodiesCache;
        _penv->GetBodiesFromEnvironmentBodyIndices(vecAttachedEnvBodyIndices, attachedBodies);
        for (const KinBodyPtr& pattachedBody : attachedBodies) {
            if (!!pattachedBody) {
                Synchronize(*pattachedBody);
            }
        }
    }
    else {
        Synchronize(body);
    }
}

FCLSpace::FCLKinBodyInfoPtr& FCLSpace::GetInfo(const KinBody &body)
{
    int envId = body.GetEnvironmentBodyIndex();
    if ( envId <= 0 ) {
        RAVELOG_WARN_FORMAT("env=%s, body %s has invalid environment id %d", _penv->GetNameId()%body.GetName()%envId);
    }
    else {
        if (envId < (int) _currentpinfo.size()) {
            return _currentpinfo.at(envId);
        }
    }

    return _currentpinfo.at(0); // invalid
}

const FCLSpace::FCLKinBodyInfoPtr& FCLSpace::GetInfo(const KinBody &body) const
{
    int envId = body.GetEnvironmentBodyIndex();
    if ( envId <= 0 ) {
        RAVELOG_WARN_FORMAT("env=%s, body %s has invalid environment id %d", _penv->GetNameId()%body.GetName()%envId);
    }
    else {
        if (envId < (int) _currentpinfo.size()) {
            return _currentpinfo.at(envId);
        }
    }

    return _currentpinfo.at(0);
}

void FCLSpace::RemoveUserData(KinBodyConstPtr pbody) {
    if( !!pbody ) {
        RAVELOG_VERBOSE(str(boost::format("FCL User data removed from env %d (userdatakey %s) : %s") % _penv->GetId() % _userdatakey % pbody->GetName()));
        const int envId = pbody->GetEnvironmentBodyIndex();
        if (envId < (int) _vecInitializedBodies.size()) {
            _vecInitializedBodies.at(envId).reset();
        }
        FCLKinBodyInfoPtr& pinfo = GetInfo(*pbody);
        if( !!pinfo ) {
            pinfo->Reset();
        }

        if( envId == 0 ) {
            RAVELOG_WARN_FORMAT("env=%s, body '%s' has bodyIndex=0, so not adding to the environment!", _penv->GetNameId()%pbody->GetName());
        }

        if (envId < (int) _currentpinfo.size()) {
            _currentpinfo.at(envId).reset();
            //RAVELOG_INFO_FORMAT("erased %d but didn't pop back, size is %d", envId%_currentpinfo.size());
        }
        if (envId < (int) _cachedpinfo.size()) {
            _cachedpinfo.at(envId).clear();
        }
    }
}

const std::vector<KinBodyConstPtr>& FCLSpace::GetEnvBodies() const {
    return _vecInitializedBodies;
}

CollisionObjectPtr FCLSpace::GetLinkBV(const KinBody::Link &link) {
    return GetLinkBV(*link.GetParent(), link.GetIndex());
}

CollisionObjectPtr FCLSpace::GetLinkBV(const OpenRAVE::KinBody::Link &link) {
    return GetLinkBV(*link.GetParent(), link.GetIndex());
}

CollisionObjectPtr FCLSpace::GetLinkBV(const OpenRAVE::KinBody &body, int index) {
    FCLKinBodyInfoPtr& pinfo = GetInfo(body);
    if( !!pinfo ) {
        return GetLinkBV(*pinfo, index);
    } else {
        RAVELOG_WARN(str(boost::format("KinBody %s is not initialized in fclspace %s, env %d")%body.GetName()%_userdatakey%_penv->GetId()));
        return CollisionObjectPtr();
    }
}

CollisionObjectPtr FCLSpace::GetLinkBV(const FCLKinBodyInfo &info, int index) {
    return info.vlinks.at(index)->linkBV.second;
}

FCLSpace::LinkInfoPtr FCLSpace::GetLinkInfo(const OpenRAVE::KinBody::Link &link) {
    return GetInfo(*link.GetParent())->vlinks.at(link.GetIndex());
}

void FCLSpace::SetIsSelfCollisionChecker(bool bIsSelfCollisionChecker)
{
    _bIsSelfCollisionChecker = bIsSelfCollisionChecker;
}

bool FCLSpace::IsSelfCollisionChecker() const
{
    return _bIsSelfCollisionChecker;
}

const MeshFactory& FCLSpace::GetMeshFactory() const {
    return _meshFactory;
}

int FCLSpace::GetEnvironmentId() const {
    return _penv->GetId();
}

void FCLSpace::_AddGeomInfoToBVHSubmodel(fcl::BVHModel<fcl::OBB>& model, OpenRAVE::KinBody::GeometryInfo const &info)
{
    const OpenRAVE::TriMesh& mesh = info._meshcollision;
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        return;
    }

    OPENRAVE_ASSERT_OP(mesh.indices.size() % 3, ==, 0);
    size_t const num_points = mesh.vertices.size();
    size_t const num_triangles = mesh.indices.size() / 3;

    std::vector<fcl::Vec3f> fcl_points(num_points);
    for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
        Vector v = info.GetTransform()*mesh.vertices[ipoint];
        fcl_points[ipoint] = fcl::Vec3f(v.x, v.y, v.z);
    }

    std::vector<fcl::Triangle> fcl_triangles(num_triangles);
    for (size_t itri = 0; itri < num_triangles; ++itri) {
        int const *const tri_indices = &mesh.indices[3 * itri];
        fcl_triangles[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
    }
    model.addSubModel(fcl_points, fcl_triangles);
}

TransformCollisionPair FCLSpace::_CreateTransformCollisionPairFromOBB(fcl::OBB const &bv) {
    CollisionGeometryPtr pbvGeom = std::make_shared<fcl::Box>(bv.extent[0]*2.0f, bv.extent[1]*2.0f, bv.extent[2]*2.0f);
    CollisionObjectPtr pbvColl = boost::make_shared<fcl::CollisionObject>(pbvGeom);
    fcl::Quaternion3f fclBvRot;
    fclBvRot.fromAxes(bv.axis);
    Vector bvRotation = ConvertQuaternion(fclBvRot);
    Vector bvTranslation = ConvertVector(bv.center());

    return std::make_pair(Transform(bvRotation, bvTranslation), pbvColl);
}

CollisionGeometryPtr FCLSpace::_CreateFCLGeomFromGeometryInfo(const KinBody::GeometryInfo &info)
{
    switch(info._type) {

    case OpenRAVE::GT_None:
        return CollisionGeometryPtr();

    case OpenRAVE::GT_CalibrationBoard:
    case OpenRAVE::GT_Box:
        return std::make_shared<fcl::Box>(info._vGeomData.x*2.0f,info._vGeomData.y*2.0f,info._vGeomData.z*2.0f);

    case OpenRAVE::GT_Sphere:
        return std::make_shared<fcl::Sphere>(info._vGeomData.x);

    case OpenRAVE::GT_Cylinder:
        return std::make_shared<fcl::Cylinder>(info._vGeomData.x, info._vGeomData.y);

    case OpenRAVE::GT_Container:
    case OpenRAVE::GT_TriMesh:
    case OpenRAVE::GT_Cage:
    {
        const OpenRAVE::TriMesh& mesh = info._meshcollision;
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            return CollisionGeometryPtr();
        }

        OPENRAVE_ASSERT_OP(mesh.indices.size() % 3, ==, 0);
        size_t const num_points = mesh.vertices.size();
        size_t const num_triangles = mesh.indices.size() / 3;

        std::vector<fcl::Vec3f> fcl_points(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
            Vector v = mesh.vertices[ipoint];
            fcl_points[ipoint] = fcl::Vec3f(v.x, v.y, v.z);
        }

        std::vector<fcl::Triangle> fcl_triangles(num_triangles);
        for (size_t itri = 0; itri < num_triangles; ++itri) {
            int const *const tri_indices = &mesh.indices[3 * itri];
            fcl_triangles[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        }

        return _meshFactory(fcl_points, fcl_triangles);
    }

    default:
        RAVELOG_WARN(str(boost::format("FCL doesn't support geom type %d")%info._type));
        return CollisionGeometryPtr();
    }
}

void FCLSpace::_Synchronize(FCLSpace::FCLKinBodyInfo& info, const OpenRAVE::KinBody& body)
{
    //KinBodyPtr pbody = info.GetBody();
    if( info.nLastStamp != body.GetUpdateStamp()) {
        info.nLastStamp = body.GetUpdateStamp();
        BOOST_ASSERT( body.GetLinks().size() == info.vlinks.size() );
        CollisionObjectPtr pcoll;
        for(size_t i = 0; i < body.GetLinks().size(); ++i) {
            const FCLSpace::FCLKinBodyInfo::LinkInfo& linkInfo = *info.vlinks[i];
            pcoll = linkInfo.linkBV.second;
            if( !pcoll ) {
                continue;
            }
            const Transform& linkTransform = body.GetLinks()[i]->GetTransform();
            Transform pose = linkTransform * linkInfo.linkBV.first;
            fcl::Vec3f newPosition = ConvertVector(pose.trans);
            fcl::Quaternion3f newOrientation = ConvertQuaternion(pose.rot);

            pcoll->setTranslation(newPosition);
            pcoll->setQuatRotation(newOrientation);
            // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
            pcoll->computeAABB();

            //info.vlinks[i]->nLastStamp = info.nLastStamp;
            for (const TransformCollisionPair& pgeom : linkInfo.vgeoms) {
                fcl::CollisionObject& coll = *pgeom.second;
                Transform pose1 = linkTransform * pgeom.first;
                fcl::Vec3f newPosition1 = ConvertVector(pose1.trans);
                fcl::Quaternion3f newOrientation1 = ConvertQuaternion(pose1.rot);

                coll.setTranslation(newPosition1);
                coll.setQuatRotation(newOrientation1);
                // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                coll.computeAABB();
            }
        }
    }
}

/// \brief controls whether the kinbody info is removed during the destructor
class FCLKinBodyInfoRemover
{
public:
    FCLKinBodyInfoRemover(const boost::function<void()>& fn) : _fn(fn) {
        _bDoRemove = true;
    }
    ~FCLKinBodyInfoRemover() {
        if( _bDoRemove ) {
            _fn();
        }
    }

    void ResetRemove() {
        _bDoRemove = false;
    }

private:
    boost::function<void()> _fn;
    bool _bDoRemove;
};

void FCLSpace::_ResetCurrentGeometryCallback(boost::weak_ptr<FCLSpace::FCLKinBodyInfo> _pinfo)
{
    FCLKinBodyInfoPtr pinfo = _pinfo.lock();
    KinBodyPtr pbody = pinfo->GetBody();
    const int bodyIndex = pbody->GetEnvironmentBodyIndex();
    if (0 < bodyIndex && bodyIndex < (int)_currentpinfo.size()) {
        const FCLKinBodyInfoPtr& pcurrentinfo = _currentpinfo.at(bodyIndex);

        if( !!pinfo && pinfo == pcurrentinfo ) {//pinfo->_geometrygroup.size() == 0 ) {
            // pinfo is current set to the current one, so should InitKinBody into _currentpinfo
            //RAVELOG_VERBOSE_FORMAT("env=%d, resetting current geometry for kinbody %s nGeometryUpdateStamp=%d, (key %s, self=%d)", _penv->GetId()%pbody->GetName()%pinfo->nGeometryUpdateStamp%_userdatakey%_bIsSelfCollisionChecker);
            pinfo->nGeometryUpdateStamp++;
            FCLKinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
            InitKinBody(pbody, pinfo, false);
            remover.ResetRemove(); // succeeded
        }
    }
}

void FCLSpace::_ResetGeometryGroupsCallback(boost::weak_ptr<FCLSpace::FCLKinBodyInfo> _pinfo)
{
    FCLKinBodyInfoPtr pinfo = _pinfo.lock();
    KinBodyPtr pbody = pinfo->GetBody();

    if( !!pinfo ) {// && pinfo->_geometrygroup.size() > 0 ) {
        //RAVELOG_VERBOSE_FORMAT("env=%d, resetting geometry groups for kinbody %s, nGeometryUpdateStamp=%d (key %s, self=%d)", _penv->GetId()%pbody->GetName()%pinfo->nGeometryUpdateStamp%_userdatakey%_bIsSelfCollisionChecker);
        pinfo->nGeometryUpdateStamp++;
        FCLKinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
        InitKinBody(pbody, pinfo, false);
        remover.ResetRemove(); // succeeded
    }
}

void FCLSpace::_ResetLinkEnableCallback(boost::weak_ptr<FCLSpace::FCLKinBodyInfo> _pinfo) {
    FCLKinBodyInfoPtr pinfo = _pinfo.lock();
    if( !!pinfo ) {
        pinfo->nLinkUpdateStamp++;
    }
}

void FCLSpace::_ResetActiveDOFsCallback(boost::weak_ptr<FCLSpace::FCLKinBodyInfo> _pinfo) {
    FCLKinBodyInfoPtr pinfo = _pinfo.lock();
    if( !!pinfo ) {
        pinfo->nActiveDOFUpdateStamp++;
    }
}

void FCLSpace::_ResetAttachedBodyCallback(boost::weak_ptr<FCLSpace::FCLKinBodyInfo> _pinfo) {
    FCLKinBodyInfoPtr pinfo = _pinfo.lock();
    if( !!pinfo ) {
        pinfo->nAttachedBodiesUpdateStamp++;
    }
}

FCLSpace::FCLKinBodyInfo::FCLKinBodyInfo()
    : nLastStamp(0)
    , nLinkUpdateStamp(0)
    , nGeometryUpdateStamp(0)
    , nAttachedBodiesUpdateStamp(0)
    , nActiveDOFUpdateStamp(0) {}

FCLSpace::FCLKinBodyInfo::~FCLKinBodyInfo() {
    Reset();
}

void FCLSpace::FCLKinBodyInfo::Reset()
{
    for (const auto& link : vlinks) {
        link->Reset();
    }
    vlinks.clear();
}

OpenRAVE::KinBodyPtr FCLSpace::FCLKinBodyInfo::GetBody()
{
    return _pbody.lock();
}

FCLSpace::FCLKinBodyInfo::FCLGeometryInfo::FCLGeometryInfo()
    : bFromKinBodyGeometry(false) {}

FCLSpace::FCLKinBodyInfo::FCLGeometryInfo::FCLGeometryInfo(OpenRAVE::KinBody::GeometryPtr pgeom)
    : _pgeom(pgeom)
    , bFromKinBodyGeometry(true) {}

FCLSpace::FCLKinBodyInfo::FCLGeometryInfo::~FCLGeometryInfo() {}

OpenRAVE::KinBody::GeometryPtr FCLSpace::FCLKinBodyInfo::FCLGeometryInfo::GetGeometry() {
    return _pgeom.lock();
}


FCLSpace::FCLKinBodyInfo::LinkInfo::LinkInfo()
    : bFromKinBodyLink(false) {}

FCLSpace::FCLKinBodyInfo::LinkInfo::LinkInfo(OpenRAVE::KinBody::LinkPtr plink)
    : _plink(plink)
    , bFromKinBodyLink(true) {
}

FCLSpace::FCLKinBodyInfo::LinkInfo::~LinkInfo() {
    Reset();
}

void FCLSpace::FCLKinBodyInfo::LinkInfo::Reset() {
    if( !!linkBV.second ) {
        if( !!GetLink() ) {
            RAVELOG_VERBOSE_FORMAT("env=%s, resetting link %s:%s col=0x%x", GetLink()->GetParent()->GetEnv()->GetNameId()%GetLink()->GetParent()->GetName()%GetLink()->GetName()%(uint64_t)linkBV.second.get());
        }
        else {
            RAVELOG_VERBOSE_FORMAT("resetting unknown link col=0x%x", (uint64_t)linkBV.second.get());
        }
        linkBV.second->setUserData(nullptr); // reset the user data since someone can hold a ref to the collision object and continue using it
    }
    linkBV.second.reset();

    for (const auto& geompair : vgeoms) {
        geompair.second->setUserData(nullptr);
        geompair.second.reset();
    }
    vgeoms.resize(0);

    // make sure to clear vgeominfos after vgeoms because the CollisionObject inside each vgeom element has a corresponding vgeominfo as a void pointer.
    vgeominfos.resize(0);
}

KinBody::LinkPtr FCLSpace::FCLKinBodyInfo::LinkInfo::GetLink() {
    return _plink.lock();
}

} // namespace