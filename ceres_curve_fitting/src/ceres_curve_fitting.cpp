#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>

std::vector<Eigen::Vector2d> GenerateData(const int num_data) {
    // y = ax^2 + bx + c (a = 3, b = 2, c = 1)
    const double a = 3., b = 2., c = 1.;
    const std::array<double, 2> interval{-10., 10.};
    const double increment = (interval.at(1) - interval.at(0)) / (num_data - 1);

    cv::RNG rng;
    std::vector<Eigen::Vector2d> data;
    for (int i = 0; i < num_data; ++i) {
        double x = interval.at(0) + i * increment;
        double y = a * x * x + b * x + c + rng.gaussian(1.0);
        data.push_back({x, y});
    }

    return data;
}

struct Residual {
    Residual(double x, double y) : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const a, const T* const b, const T* const c,
                    T* residual) const {
        residual[0] = y_ - (a[0] * x_ * x_ + b[0] * x_ + c[0]);
        return true;
    }

  private:
    const double x_;
    const double y_;
};

int main() {
    const int num_data = 100;
    std::vector<Eigen::Vector2d> data = GenerateData(num_data);

    double a = 0., b = 0., c = 0.;

    ceres::Problem problem;
    for (int i = 0; i < num_data; ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<Residual, 1, 1, 1, 1>(
                new Residual(data.at(i).x(), data.at(i).y()));

        problem.AddResidualBlock(cost_function, nullptr, &a, &b, &c);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Initial a: " << 0.0 << ", b: " << 0.0 << ", c: " << 0.0
              << std::endl;
    std::cout << "Final   a: " << a << ", b: " << b << ", c: " << c
              << std::endl;

    return 0;
}