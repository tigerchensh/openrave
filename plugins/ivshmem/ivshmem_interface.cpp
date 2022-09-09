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
