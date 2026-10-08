#include "thrust_allocator_c.h"
#include "ThrustAllocationCore.hpp"

#include <Eigen/Dense>
#include <new>

struct TA_Handle
{
    ThrustAllocationCore solver;
};

extern "C"
{

TA_Handle* ta_create(void)
{
    try
    {
        return new TA_Handle();
    }
    catch (...)
    {
        return nullptr;
    }
}

void ta_destroy(TA_Handle* handle)
{
    delete handle;
}

int ta_add_thruster(
    TA_Handle* handle,
    double px, double py, double pz,
    double ax, double ay, double az,
    double max_thrust,
    double torque_direction,
    double reaction_torque)
{
    if (!handle)
        return 0;

    return handle->solver.addThruster(
               Eigen::Vector3d(px, py, pz),
               Eigen::Vector3d(ax, ay, az),
               max_thrust,
               torque_direction,
               reaction_torque)
               ? 1
               : 0;
}

int ta_set_wrench_weights(
    TA_Handle* handle,
    const double weights6[6])
{
    if (!handle || !weights6)
        return 0;

    ThrustAllocationCore::WrenchVector w;
    for (int i = 0; i < 6; ++i)
        w(i) = weights6[i];

    return handle->solver.setWrenchWeights(w)
               ? 1
               : 0;
}

int ta_set_effort_weight(
    TA_Handle* handle,
    double weight)
{
    if (!handle)
        return 0;

    return handle->solver.setEffortWeight(weight)
               ? 1
               : 0;
}

int ta_initialize(TA_Handle* handle)
{
    if (!handle)
        return 0;

    return handle->solver.initialize() ? 1 : 0;
}

static int solve_common(
    TA_Handle* handle,
    const double desired6[6],
    double* output,
    size_t out_capacity,
    bool actual_thrust)
{
    if (!handle || !desired6 || !output)
        return 0;

    const size_t count =
        static_cast<size_t>(
            handle->solver.getThrusterCount());

    if (out_capacity < count)
        return 0;

    Eigen::VectorXd desired(6);
    for (int i = 0; i < 6; ++i)
        desired(i) = desired6[i];

    Eigen::VectorXd result;

    const bool ok =
        actual_thrust
            ? handle->solver.solveThrust(
                  desired, result)
            : handle->solver.solve(
                  desired, result);

    if (!ok)
        return 0;

    for (size_t i = 0; i < count; ++i)
        output[i] = result(
            static_cast<Eigen::Index>(i));

    return 1;
}

int ta_solve_normalized(
    TA_Handle* handle,
    const double desired6[6],
    double* normalized_out,
    size_t out_capacity)
{
    return solve_common(
        handle,
        desired6,
        normalized_out,
        out_capacity,
        false);
}

int ta_solve_thrust(
    TA_Handle* handle,
    const double desired6[6],
    double* thrust_out,
    size_t out_capacity)
{
    return solve_common(
        handle,
        desired6,
        thrust_out,
        out_capacity,
        true);
}

size_t ta_get_thruster_count(
    const TA_Handle* handle)
{
    if (!handle)
        return 0;

    return static_cast<size_t>(
        handle->solver.getThrusterCount());
}

int ta_is_initialized(
    const TA_Handle* handle)
{
    if (!handle)
        return 0;

    return handle->solver.isInitialized()
               ? 1
               : 0;
}

} // extern "C"
