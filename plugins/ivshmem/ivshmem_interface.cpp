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

#include <cstdarg>
#include <memory>
#include <typeinfo>

#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/shape/geometric_shapes.h>

#include "ravefcl.hpp"
#include "ivshmem_interface.hpp"

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

//using FCLObjectTypes = std::variant<
//    fcl::Box,
//    fcl::Sphere,
//    fcl::Cylinder,
//    //fcl::Convex,
//    //fcl::Halfspace,
//    //fcl::Plane,
//    fcl::BVHModel<fcl::AABB>,
//    fcl::BVHModel<fcl::OBB>,
//    fcl::BVHModel<fcl::RSS>,
//    fcl::BVHModel<fcl::OBBRSS>,
//    fcl::BVHModel<fcl::KDOP<16>>,
//    fcl::BVHModel<fcl::KDOP<18>>,
//    fcl::BVHModel<fcl::KDOP<24>>,
//    fcl::BVHModel<fcl::kIOS>,
//>;

inline std::unique_ptr<fcl::CollisionGeometry> ConstructFCLMesh(const OpenRAVE::KinBody::GeometryInfo& info) {
    switch (info._type) {
    case OpenRAVE::GT_None: {
        break;
    }
    case OpenRAVE::GT_CalibrationBoard:
    case OpenRAVE::GT_Box: {
        return std::make_unique<fcl::Box>(info._vGeomData.x * 2.f, info._vGeomData.y * 2.f, info._vGeomData.z * 2.f);
        break;
    }
    case OpenRAVE::GT_Sphere: {
        return std::make_unique<fcl::Sphere>(info._vGeomData.x);
        break;
    }
    case OpenRAVE::GT_Cylinder: {
        return std::make_unique<fcl::Cylinder>(info._vGeomData.x, info._vGeomData.y);
        break;
    }
    case OpenRAVE::GT_Container:
    case OpenRAVE::GT_TriMesh:
    case OpenRAVE::GT_Cage: {
        return ConvertMeshToFCL(info._meshcollision);
        break;
    }
    default: {
        RAVELOG_WARN("Unsupported geometry type.");
    }
    }
    return nullptr;
}


IVShMemInterface::IVShMemInterface(OpenRAVE::EnvironmentBasePtr penv)
    : OpenRAVE::CollisionCheckerBase(penv)
    , _ivshmem_server()
    , _ivshmem_server_thread(&IVShMemServer::Thread, std::ref(_ivshmem_server)) {
}

IVShMemInterface::~IVShMemInterface() {
    _ivshmem_server.Stop();
    _ivshmem_server_thread.join();
}

bool IVShMemInterface::SetCollisionOptions(int collisionoptions) {
    _options = collisionoptions;
    return !(_options & OpenRAVE::CO_RayAnyHit); // Ray collisions not supported.
}

int IVShMemInterface::GetCollisionOptions() const {
    return _options;
}

void IVShMemInterface::SetTolerance(OpenRAVE::dReal tolerance) {}

bool IVShMemInterface::InitEnvironment() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    GetEnv()->GetBodies(bodies);
    for (auto& body : bodies) {
        if (!InitKinBody(body)) {
            return false;
        }
    }
    return true;
}

void IVShMemInterface::DestroyEnvironment() {}

bool IVShMemInterface::InitKinBody(OpenRAVE::KinBodyPtr pbody) {
    const auto& links = pbody->GetLinks();
    for (const auto& linkptr : links) {
        fcl::AABB enclosingBV;
        const auto& geometries = linkptr->GetGeometries();
        for (const auto& geometryptr : geometries) {
            const auto& info = geometryptr->GetInfo();
            std::unique_ptr<fcl::CollisionGeometry> collisionGeometry = ConstructFCLMesh(info);
            if (!collisionGeometry) {
                continue;
            }
        }
    }
    return true;
}

void IVShMemInterface::RemoveKinBody(OpenRAVE::KinBodyPtr pbody) {
}




bool IVShMemInterface::CheckCollision(const OpenRAVE::RAY&, OpenRAVE::KinBody::LinkConstPtr, OpenRAVE::CollisionReportPtr) {
    RAVELOG_WARN("Ray collisions are not supported.\n");
    return false;
}

bool IVShMemInterface::CheckCollision(const OpenRAVE::RAY&, OpenRAVE::KinBodyConstPtr, OpenRAVE::CollisionReportPtr) {
    RAVELOG_WARN("Ray collisions are not supported.\n");
    return false;
}

bool IVShMemInterface::CheckCollision(const OpenRAVE::RAY&, OpenRAVE::CollisionReportPtr) {
    RAVELOG_WARN("Ray collisions are not supported.\n");
    return false;
}
