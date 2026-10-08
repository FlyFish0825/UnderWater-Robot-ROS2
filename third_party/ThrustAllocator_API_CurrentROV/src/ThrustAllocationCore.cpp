#include "ThrustAllocationCore.hpp"

#include <cmath>
#include <iostream>

ThrustAllocationCore::ThrustAllocationCore()
    : num_thrusters(0),
      num_dof(6),
      num_constraints(0),
      wrench_weights(WrenchVector::Ones()),
      effort_weight(1e-6),
      is_initialized(false)
{
}

bool ThrustAllocationCore::addThruster(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& axis,
    double max_thrust,
    double k_direction,
    double k_torque)
{
    if (is_initialized)
        return false;

    if (!position.allFinite() ||
        !axis.allFinite() ||
        !std::isfinite(max_thrust) ||
        !std::isfinite(k_direction) ||
        !std::isfinite(k_torque))
        return false;

    if (axis.squaredNorm() < 1e-12 || max_thrust <= 0.0)
        return false;

    ThrusterConfig config;
    config.position = position;
    config.axis = axis.normalized();
    config.max_thrust = max_thrust;
    config.k_direction = k_direction;
    config.k_torque = k_torque;

    thrusters.push_back(config);
    return true;
}

bool ThrustAllocationCore::setWrenchWeights(
    const WrenchVector& weights)
{
    if (is_initialized)
        return false;

    if (!weights.allFinite() ||
        (weights.array() <= 0.0).any())
        return false;

    wrench_weights = weights;
    return true;
}

bool ThrustAllocationCore::setEffortWeight(double weight)
{
    if (is_initialized ||
        !std::isfinite(weight) ||
        weight <= 0.0)
        return false;

    effort_weight = weight;
    return true;
}

bool ThrustAllocationCore::initialize()
{
    if (is_initialized)
        return true;

    num_thrusters = static_cast<int>(thrusters.size());
    if (num_thrusters <= 0)
        return false;

    B.resize(num_dof, num_thrusters);

    for (int i = 0; i < num_thrusters; ++i)
    {
        const auto& t = thrusters[i];

        const Eigen::Vector3d force =
            t.max_thrust * t.axis;

        Eigen::Vector3d torque =
            t.position.cross(force);

        torque +=
            t.k_direction *
            t.k_torque *
            t.axis;

        B.block<3, 1>(0, i) = force;
        B.block<3, 1>(3, i) = torque;
    }

    const Eigen::Matrix<double, 6, 6> W =
        wrench_weights.asDiagonal();

    Eigen::MatrixXd P_dense =
        2.0 *
        (B.transpose() * W * B +
         effort_weight *
             Eigen::MatrixXd::Identity(
                 num_thrusters,
                 num_thrusters));

    Eigen::SparseMatrix<double> P =
        P_dense.sparseView();

    Eigen::VectorXd q =
        Eigen::VectorXd::Zero(num_thrusters);

    num_constraints = num_thrusters;

    Eigen::SparseMatrix<double> A(
        num_constraints,
        num_thrusters);

    A.setIdentity();

    Eigen::VectorXd lower =
        Eigen::VectorXd::Constant(
            num_thrusters, -1.0);

    Eigen::VectorXd upper =
        Eigen::VectorXd::Constant(
            num_thrusters, 1.0);

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-5);
    solver.settings()->setRelativeTolerance(1e-5);

    solver.data()->setNumberOfVariables(num_thrusters);
    solver.data()->setNumberOfConstraints(num_constraints);

    if (!solver.data()->setHessianMatrix(P))
        return false;
    if (!solver.data()->setGradient(q))
        return false;
    if (!solver.data()->setLinearConstraintsMatrix(A))
        return false;
    if (!solver.data()->setLowerBound(lower))
        return false;
    if (!solver.data()->setUpperBound(upper))
        return false;
    if (!solver.initSolver())
        return false;

    is_initialized = true;
    return true;
}

bool ThrustAllocationCore::solve(
    const Eigen::VectorXd& desired,
    Eigen::VectorXd& k_solution)
{
    if (!is_initialized ||
        desired.size() != num_dof ||
        !desired.allFinite())
        return false;

    const Eigen::Matrix<double, 6, 6> W =
        wrench_weights.asDiagonal();

    Eigen::VectorXd q =
        -2.0 *
        B.transpose() *
        W *
        desired;

    if (!solver.updateGradient(q))
        return false;

    if (solver.solveProblem() !=
        OsqpEigen::ErrorExitFlag::NoError)
        return false;

    const auto status = solver.getStatus();

    if (status != OsqpEigen::Status::Solved &&
        status != OsqpEigen::Status::SolvedInaccurate)
        return false;

    k_solution = solver.getSolution();

    if (k_solution.size() != num_thrusters ||
        !k_solution.allFinite())
        return false;

    k_solution =
        k_solution.cwiseMax(-1.0).cwiseMin(1.0);

    return true;
}

bool ThrustAllocationCore::solveThrust(
    const Eigen::VectorXd& desired,
    Eigen::VectorXd& thrust_solution)
{
    Eigen::VectorXd k;

    if (!solve(desired, k))
    {
        thrust_solution =
            Eigen::VectorXd::Zero(num_thrusters);
        return false;
    }

    thrust_solution.resize(num_thrusters);

    for (int i = 0; i < num_thrusters; ++i)
    {
        thrust_solution(i) =
            k(i) * thrusters[i].max_thrust;
    }

    return true;
}

int ThrustAllocationCore::getThrusterCount() const
{
    return static_cast<int>(thrusters.size());
}

bool ThrustAllocationCore::isInitialized() const
{
    return is_initialized;
}
