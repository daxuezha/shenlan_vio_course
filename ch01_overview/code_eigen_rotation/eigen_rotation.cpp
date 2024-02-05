#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.hpp> // 修改为 #include <sophus/so3.hpp>，否则编译报错

/**
 * @brief Eigen::UnitRandom(), ref: http://planning.cs.uiuc.edu/node198.html
 * @return
 */
Eigen::Quaterniond unit_random() {
    double u1 = rand() / double(RAND_MAX); // [0, 1]
    double u2 = rand() / double(RAND_MAX) * M_2_PI;
    double u3 = rand() / double(RAND_MAX) * M_2_PI;
    double a = std::sqrt(1 - u1);
    double b = std::sqrt(u1);
    return Eigen::Quaterniond(a*sin(u2), a*cos(u2), b*sin(u3), b*cos(u3)).normalized();
}

int main()
{
    // Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Eigen::Matrix3d R(unit_random());
    std::cout << "the random initial rotation matrix R:\n" << R << std::endl << std::endl;

    Sophus::SO3<double> R_SO3(R); // 这部分代码之前少了个<double>，导致编译报错
    Eigen::Vector3d omega(0.01, 0.02, 0.03);
    // 下面之前少了个<double>，导致编译报错
    Sophus::SO3<double> R_SO3_updated = R_SO3 * Sophus::SO3<double>::exp(omega);
    Eigen::Matrix3d R1 = R_SO3_updated.matrix();
    std::cout << "R1:\n" << R1 << std::endl << std::endl;

    Eigen::Quaterniond R_q(R);
    Eigen::Quaterniond q_update(1, omega[0]*0.5, omega[1]*0.5, omega[2]*0.5);
    Eigen::Quaterniond R_q_updated = (R_q*q_update).normalized();
    Eigen::Matrix3d R2 = R_q_updated.toRotationMatrix();
    std::cout << "R2:\n" << R2 << std::endl << std::endl;

    Eigen::Matrix3d R_error = R1 - R2;
    std::cout << "error matrix (R1-R2):\n" << R_error << std::endl << std::endl;

    double error_f_norm = R_error.norm();
    std::cout << "the Frobenius Norm of the error matrix: " << error_f_norm << std::endl;

    return 0;
}

