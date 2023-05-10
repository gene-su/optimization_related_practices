#include <ceres/ceres.h>

#include <iostream>

#include "bal_problem.h"
#include "snavely_reprojection_error.h"

int main(int argc, char** argv) {
    BALProblem bal_problem;
    bal_problem.LoadFile("../data/problem-16-22106-pre.txt");
    bal_problem.WriteToPLYFile("before_ba.ply");

    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        const double* observations = bal_problem.observations();
        ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(
            observations[2 * i + 0], observations[2 * i + 1]);

        problem.AddResidualBlock(cost_function, nullptr,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i));
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    bal_problem.WriteToPLYFile("after_ba.ply");

    return 0;
}