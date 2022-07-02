#include "runtime/function/controller/character_controller.h"

#include "runtime/core/base/macro.h"

#include "runtime/function/framework/component/motor/motor_component.h"
#include "runtime/function/framework/world/world_manager.h"
#include "runtime/function/global/global_context.h"
#include "runtime/function/physics/physics_scene.h"

namespace Pilot
{
    CharacterController::CharacterController(const Capsule& capsule) : m_capsule(capsule)
    {
        m_rigidbody_shape                                    = RigidBodyShape();
        m_rigidbody_shape.m_geometry                         = PILOT_REFLECTION_NEW(Capsule);
        *static_cast<Capsule*>(m_rigidbody_shape.m_geometry) = m_capsule;

        m_rigidbody_shape.m_type = RigidBodyShapeType::capsule;

        Quaternion orientation;
        orientation.fromAngleAxis(Radian(Degree(90.f)), Vector3::UNIT_X);

        m_rigidbody_shape.m_local_transform =
            Transform(
                Vector3(0, 0, capsule.m_half_height + capsule.m_radius),
                orientation,
                Vector3::UNIT_SCALE);
    }

    Vector3 CharacterController::move(const Vector3& current_position, const Vector3& displacement)
    {
        
        std::shared_ptr<PhysicsScene> physics_scene =
            g_runtime_global_context.m_world_manager->getCurrentActivePhysicsScene().lock();
        ASSERT(physics_scene);

        std::vector<PhysicsHitInfo> hits;

        Transform world_transform = Transform(
            current_position + 0.1f * Vector3::UNIT_Z,
            Quaternion::IDENTITY,
            Vector3::UNIT_SCALE);

        Vector3 vertical_displacement   = displacement.z * Vector3::UNIT_Z;
        Vector3 horizontal_displacement = Vector3(displacement.x, displacement.y, 0.f);

        Vector3 vertical_direction   = vertical_displacement.normalisedCopy();
        Vector3 horizontal_direction = horizontal_displacement.normalisedCopy();

        Vector3 final_position = current_position;

        m_is_touch_ground = is_climb_stair || is_down_stair? true: physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            Vector3::NEGATIVE_UNIT_Z,
            0.105f,
            hits);
        hits.clear();
        
        world_transform.m_position -= 0.1f * Vector3::UNIT_Z;
        // vertical pass
        if (physics_scene->sweep(m_rigidbody_shape,
                                 world_transform.getMatrix(),
                                 vertical_direction,
                                 vertical_displacement.length(),
                                 hits))
        {
            final_position += hits[0].hit_distance * vertical_direction;
        }
        else
        {
            final_position += vertical_displacement;
        }
        hits.clear(); 

        // side pass
        if (physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            horizontal_direction,
            horizontal_displacement.length(),
            hits))
        {
            // get maximum hit point                                 
            auto  max_hit_point  = std::max_element(
                hits.begin(), hits.end(), [](auto& e1, auto& e2) { 
                    return e1.hit_position.z < e2.hit_position.z; 
                });            

            // check whether able to climb
            if (float delta = max_hit_point->hit_position.z - world_transform.m_position.z;
                vertical_displacement.length() < std::numeric_limits<float>::epsilon() && 
                delta > 0.0f && delta < 0.285f)
            {
                is_climb_stair     = true;                
                climb_count        = 0;
                final_position.z  += 0.25f * delta;                                
            }
            // not climb then remove the collision part
            else
            {
                is_climb_stair     = false;
                climb_count        = 0;
                for (auto& hit : hits)
                {
                    Vector3 hit_dir(hit.hit_normal.x, hit.hit_normal.y, 0.0f);
                    horizontal_displacement -= horizontal_displacement.dotProduct(hit.hit_normal) * hit_dir;
                }
            }
        }
        else
        {            
            // wating next move and adding climbing count
            if (is_climb_stair)
            {
                ++climb_count;
                // not climbing
                if (climb_count > 10)
                {
                    climb_count    = 0;
                    is_climb_stair = false;
                }
            }
            // not climbing stairs check whether going down stairs
            else if (vertical_displacement.length() < std::numeric_limits<float>::epsilon() &&
                     physics_scene->raycast(final_position, Vector3::NEGATIVE_UNIT_Z, 0.50f, hits))
            {
                is_down_stair         = true;
                auto min_height_point = std::min_element(hits.begin(), hits.end(), [](auto& e1, auto& e2) {
                    return e1.hit_position.z > e2.hit_position.z;
                });
                if (min_height_point->hit_distance < 0.50f)
                {
                    final_position.z -= min_height_point->hit_distance * 0.1f;
                }
            }            
            else
            {
                is_down_stair        = false;
            }
        }
        final_position += horizontal_displacement;
        hits.clear();
              
        return final_position;
    }

} // namespace Pilot
