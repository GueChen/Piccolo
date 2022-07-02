#pragma once

#include "runtime/core/math/vector3.h"
#include "runtime/resource/res_type/components/rigid_body.h"
#include "runtime/resource/res_type/data/basic_shape.h"

namespace Pilot
{
    enum SweepPass
    {
        SWEEP_PASS_UP,
        SWEEP_PASS_SIDE,
        SWEEP_PASS_DOWN,
        SWEEP_PASS_SENSOR
    };

    class Controller
    {
    public:
        virtual ~Controller() = default;

        virtual Vector3 move(const Vector3& current_position, const Vector3& displacement) = 0;

        bool isTouchGround() const { return m_is_touch_ground; }

    protected:
        bool m_is_touch_ground {false};
    };

    class CharacterController : public Controller
    {
    public:
        CharacterController(const Capsule& capsule);

        Vector3 move(const Vector3& current_position, const Vector3& displacement) override;

    private:
        Capsule        m_capsule;
        RigidBodyShape m_rigidbody_shape;

        // flags for climbing and going down staris status
        bool is_climb_stair = false;
        bool is_down_stair  = false;
        int  climb_count    = 0;
    };
} // namespace Pilot
