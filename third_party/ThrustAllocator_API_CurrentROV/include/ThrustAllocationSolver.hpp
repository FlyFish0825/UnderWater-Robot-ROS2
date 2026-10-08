#ifndef THRUST_ALLOCATION_SOLVER_PUBLIC_HPP
#define THRUST_ALLOCATION_SOLVER_PUBLIC_HPP

#include "thrust_allocator_c.h"

#include <array>
#include <cstddef>
#include <utility>

/*
 * Public C++ wrapper.
 *
 * IMPORTANT:
 * This header intentionally contains NO Eigen / OSQP types.
 * Application code only needs this header and the compiled library.
 */
class ThrustAllocationSolver
{
public:
    struct Vec3
    {
        double x;
        double y;
        double z;

        constexpr Vec3(double x_ = 0.0,
                       double y_ = 0.0,
                       double z_ = 0.0) noexcept
            : x(x_), y(y_), z(z_)
        {
        }
    };

    using Wrench = std::array<double, 6>;

    ThrustAllocationSolver()
        : handle_(ta_create())
    {
    }

    ~ThrustAllocationSolver()
    {
        ta_destroy(handle_);
    }

    ThrustAllocationSolver(const ThrustAllocationSolver&) = delete;
    ThrustAllocationSolver& operator=(const ThrustAllocationSolver&) = delete;

    ThrustAllocationSolver(ThrustAllocationSolver&& other) noexcept
        : handle_(other.handle_)
    {
        other.handle_ = nullptr;
    }

    ThrustAllocationSolver& operator=(ThrustAllocationSolver&& other) noexcept
    {
        if (this != &other)
        {
            ta_destroy(handle_);
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    bool valid() const noexcept
    {
        return handle_ != nullptr;
    }

    bool addThruster(const Vec3& position,
                     const Vec3& axis,
                     double max_thrust,
                     double torque_direction = 1.0,
                     double reaction_torque = 0.0)
    {
        return ta_add_thruster(
                   handle_,
                   position.x, position.y, position.z,
                   axis.x, axis.y, axis.z,
                   max_thrust,
                   torque_direction,
                   reaction_torque) != 0;
    }

    bool setWrenchWeights(const Wrench& weights)
    {
        return ta_set_wrench_weights(handle_, weights.data()) != 0;
    }

    bool setEffortWeight(double weight)
    {
        return ta_set_effort_weight(handle_, weight) != 0;
    }

    bool initialize()
    {
        return ta_initialize(handle_) != 0;
    }

    std::size_t getThrusterCount() const noexcept
    {
        return ta_get_thruster_count(handle_);
    }

    bool isInitialized() const noexcept
    {
        return ta_is_initialized(handle_) != 0;
    }

    bool solveNormalized(const Wrench& desired,
                         double* output,
                         std::size_t output_capacity)
    {
        return ta_solve_normalized(
                   handle_,
                   desired.data(),
                   output,
                   output_capacity) != 0;
    }

    bool solveThrust(const Wrench& desired,
                     double* output,
                     std::size_t output_capacity)
    {
        return ta_solve_thrust(
                   handle_,
                   desired.data(),
                   output,
                   output_capacity) != 0;
    }

    bool solveThrust(double Fx,
                     double Fy,
                     double Fz,
                     double Mx,
                     double My,
                     double Mz,
                     double* output,
                     std::size_t output_capacity)
    {
        const Wrench desired{Fx, Fy, Fz, Mx, My, Mz};
        return solveThrust(desired, output, output_capacity);
    }

    template <std::size_t N>
    bool solveNormalized(const Wrench& desired,
                         std::array<double, N>& output)
    {
        return solveNormalized(desired, output.data(), output.size());
    }

    template <std::size_t N>
    bool solveThrust(const Wrench& desired,
                     std::array<double, N>& output)
    {
        return solveThrust(desired, output.data(), output.size());
    }

    template <std::size_t N>
    bool solveThrust(double Fx,
                     double Fy,
                     double Fz,
                     double Mx,
                     double My,
                     double Mz,
                     std::array<double, N>& output)
    {
        return solveThrust(
            Fx, Fy, Fz, Mx, My, Mz,
            output.data(), output.size());
    }

private:
    TA_Handle* handle_ = nullptr;
};

#endif
