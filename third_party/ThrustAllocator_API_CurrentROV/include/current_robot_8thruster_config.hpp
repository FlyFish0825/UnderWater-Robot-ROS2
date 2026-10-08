#ifndef CURRENT_ROBOT_8THRUSTER_CONFIG_HPP
#define CURRENT_ROBOT_8THRUSTER_CONFIG_HPP

#include "ThrustAllocationSolver.hpp"

#include <array>
#include <cstddef>

namespace current_rov
{
struct ThrusterGeometry
{
    ThrustAllocationSolver::Vec3 position_m;
    ThrustAllocationSolver::Vec3 axis_base;
};

inline constexpr std::array<ThrusterGeometry, 8> kThrusters{{
    {{ 0.230160, -0.183590, -0.040532}, {-0.70710678, -0.70710678, 0.0}},
    {{ 0.230160,  0.183590, -0.040532}, {-0.70710678,  0.70710678, 0.0}},
    {{-0.255910, -0.183590, -0.040532}, { 0.70710678, -0.70710678, 0.0}},
    {{-0.255910,  0.183590, -0.040532}, { 0.70710678,  0.70710678, 0.0}},
    {{ 0.062129, -0.125000, -0.040532}, { 0.0, 0.0, 1.0}},
    {{ 0.062129,  0.125000, -0.040532}, { 0.0, 0.0, 1.0}},
    {{-0.087871, -0.125000, -0.040532}, { 0.0, 0.0, 1.0}},
    {{-0.087871,  0.125000, -0.040532}, { 0.0, 0.0, 1.0}}
}};

/* Geometry extracted from ROV_Product_FV.SLDASM URDF. * max_thrust_N is NOT in the URDF and must come from propulsion data.
 * reaction_torque_Nm also defaults to 0 because URDF does not define it.
 */
inline bool addThrusters(
    ThrustAllocationSolver& allocator,
    double max_thrust_N,
    double reaction_torque_Nm = 0.0)
{
    for (std::size_t i = 0; i < kThrusters.size(); ++i)
    {
        const auto& t = kThrusters[i];
        if (!allocator.addThruster(
                t.position_m,
                t.axis_base,
                max_thrust_N,
                1.0,
                reaction_torque_Nm))
        {
            return false;
        }
    }
    return true;
}
} // namespace current_rov

#endif
