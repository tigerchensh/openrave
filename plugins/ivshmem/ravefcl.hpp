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

#ifndef OPENRAVE_RAVEFCL_HPP
#define OPENRAVE_RAVEFCL_HPP

#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/BVH/BVH_model.h>

#include <openrave/openrave.h>

// Conversion functions between OpenRAVE and FCL data.

inline fcl::Vec3f ConvertVector(const OpenRAVE::Vector& v) {
    return fcl::Vec3f(v.x, v.y, v.z);
}

inline OpenRAVE::Vector ConvertVector(const fcl::Vec3f& v) {
    return OpenRAVE::Vector(v[0], v[1], v[2]);
}

inline fcl::Quaternion3f ConvertQuaternion(const OpenRAVE::Vector& q) {
    return fcl::Quaternion3f(q.w, q.x, q.y, q.z);
}

inline OpenRAVE::Vector ConvertQuaternion(const fcl::Quaternion3f& q) {
    return OpenRAVE::Vector(q[0], q[1], q[2], q[3]);
}

inline fcl::AABB ConvertAABB(const OpenRAVE::AABB& aabb) {
    return fcl::AABB(ConvertVector(aabb.pos))
        .expand(ConvertVector(aabb.extents));
}

inline std::unique_ptr<fcl::CollisionGeometry> ConvertMeshToFCL(const OpenRAVE::TriMesh& mesh) {
    OPENRAVE_ASSERT_OP(mesh.indices.size() % 3, ==, 0);
    const auto& vertices = mesh.vertices;
    const auto& indices = mesh.indices;
    auto model = std::make_unique<fcl::BVHModel<fcl::OBB>>();
    model->beginModel();
    for (size_t i = 0; i < indices.size(); i += 3) {
        model->addTriangle(
            ConvertVector(vertices[indices[i + 0]]),
            ConvertVector(vertices[indices[i + 1]]),
            ConvertVector(vertices[indices[i + 2]])
        );
    }
    model->endModel();
    return model;
}

#endif // OPENRAVE_RAVEFCL_HPP