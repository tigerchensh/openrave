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

#ifndef OPENRAVE_IVSHMEM_INTERFACE_HPP
#define OPENRAVE_IVSHMEM_INTERFACE_HPP

#include <string>
#include <thread>

#include <openrave/openrave.h>

#include "ivshmem_server.hpp"

class IVShMemInterface final : public OpenRAVE::CollisionCheckerBase {
public:
    static constexpr char* ModuleName = "ivshmem";

    IVShMemInterface(OpenRAVE::EnvironmentBasePtr penv);
    IVShMemInterface(const IVShMemInterface&) = delete;
    ~IVShMemInterface() override;

    bool SetCollisionOptions(int collisionoptions) override;

    /// \brief get the current collision options
    int GetCollisionOptions() const override;

    void SetTolerance(OpenRAVE::dReal tolerance) override;

    /// \brief initialize the checker with the current environment and gather all current bodies in the environment and put them in its collision space
    bool InitEnvironment() override;

    /// \brief clear/deallocate any memory associated with tracking collision data for bodies
    void DestroyEnvironment() override;

    /// \brief notified when a new body has been initialized in the environment
    bool InitKinBody(OpenRAVE::KinBodyPtr pbody) override;

    /// \brief notified when a body has been removed from the environment
    void RemoveKinBody(OpenRAVE::KinBodyPtr pbody) override;

    /// Each function takes an optional pointer to a CollisionReport structure and returns true if collision occurs.
    /// \name Collision specific functions.
    /// \anchor collision_checking
    //@{

    /// \brief checks collision of a body and a scene. Attached bodies are respected. If CO_ActiveDOFs is set, will only check affected links of the body.
    bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief checks collision between two bodies. Attached bodies are respected. If CO_ActiveDOFs is set, will only check affected links of the pbody1.
    bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief checks collision of a link and a scene. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief checks collision of two links. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief checks collision of a link and a body. Attached bodies for pbody are respected. CO_ActiveDOFs option is ignored.
    bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief checks collision of a link and a scene. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, const std::vector<OpenRAVE::KinBodyConstPtr>& vbodyexcluded, const std::vector<OpenRAVE::KinBody::LinkConstPtr>& vlinkexcluded, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief checks collision of a body and a scene. Attached bodies are respected. If CO_ActiveDOFs is set, will only check affected links of pbody.
    bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody, const std::vector<OpenRAVE::KinBodyConstPtr>& vbodyexcluded, const std::vector<OpenRAVE::KinBody::LinkConstPtr>& vlinkexcluded, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief Check collision with a link and a ray with a specified length. CO_ActiveDOFs option is ignored.
    ///
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param plink the link to collide with
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    bool CheckCollision(const OpenRAVE::RAY& ray, OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief Check collision with a link and a ray with a specified length.
    ///
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param pbody the link to collide with. If CO_ActiveDOFs is set, will only check affected links of the body.
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    bool CheckCollision(const OpenRAVE::RAY& ray, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief Check collision with a body and a ray with a specified length. CO_ActiveDOFs option is ignored.
    ///
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param pbody the kinbody to look for collisions
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    bool CheckCollision(const OpenRAVE::RAY& ray, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief Checks self collision only with the links of the passed in body.
    ///
    /// Only checks KinBody::GetNonAdjacentLinks(), Links that are joined together are ignored.
    /// \param[in] pbody The body to check self-collision for
    /// \param[out] report [optional] collision report to be filled with data about the collision.
    bool CheckStandaloneSelfCollision(OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

    /// \brief Checks self collision of the link with the rest of the links with its parent
    ///
    /// Only checks KinBody::GetNonAdjacentLinks(), Links that are joined together are ignored.
    /// \param[out] report [optional] collision report to be filled with data about the collision.
    bool CheckStandaloneSelfCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()) override;

private:
    int _options; // CollisionOptions enum
    IVShMemServer _ivshmem_server;
    std::thread _ivshmem_server_thread;
};

#endif // OPENRAVE_IVSHMEM_INTERFACE_HPP