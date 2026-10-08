#ifndef THRUST_ALLOCATION_CORE_HPP
#define THRUST_ALLOCATION_CORE_HPP

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

#include <vector>

class ThrustAllocationCore
{
public:
    using WrenchVector = Eigen::Matrix<double, 6, 1>;

    ThrustAllocationCore();

    bool addThruster(const Eigen::Vector3d& position,
                     const Eigen::Vector3d& axis,
                     double max_thrust,
                     double k_direction = 1.0,
                     double k_torque = 0.0);

    bool setWrenchWeights(const WrenchVector& weights);
    bool setEffortWeight(double weight);

    bool initialize();

    bool solve(const Eigen::VectorXd& desired,
               Eigen::VectorXd& k_solution);

    bool solveThrust(const Eigen::VectorXd& desired,
                     Eigen::VectorXd& thrust_solution);

    int getThrusterCount() const;
    bool isInitialized() const;

private:
    struct ThrusterConfig
    {
        Eigen::Vector3d position;
        Eigen::Vector3d axis;
        double max_thrust;
        double k_direction;
        double k_torque;
    };

    int num_thrusters;
    const int num_dof;
    int num_constraints;

    std::vector<ThrusterConfig> thrusters;
    Eigen::MatrixXd B;

    WrenchVector wrench_weights;
    double effort_weight;

    OsqpEigen::Solver solver;
    bool is_initialized;
};

#endif
