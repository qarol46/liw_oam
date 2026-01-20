#include "poseParameterization.h"

bool PoseParameterization::Plus(const double* x, const double* delta, double* x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = numType::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}

bool PoseParameterization::ComputeJacobian(const double* x, double* jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    j.topRows<6>().setIdentity(); // Матрица 6x6 с единицами на диагонали
    // Последняя строка (7-я) уже нулевая благодаря setZero()

    return true;
}

bool RotationParameterization::Plus(const double* x, const double* delta, double* x_plus_delta) const
{
    Eigen::Map<const Eigen::Quaterniond> _q(x);

    Eigen::Quaterniond dq = numType::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));

    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);

    q = (_q * dq).normalized();

    return true;
}

bool RotationParameterization::ComputeJacobian(const double* x, double* jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    j.topRows<3>().setIdentity();
    return true;
}