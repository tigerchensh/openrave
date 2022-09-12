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

#include "ivshmem_interface.hpp"

IVShMemInterface::IVShMemInterface(OpenRAVE::EnvironmentBasePtr penv)
    : OpenRAVE::CollisionCheckerBase(penv)
    {}

IVShMemInterface::~IVShMemInterface() {}

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
