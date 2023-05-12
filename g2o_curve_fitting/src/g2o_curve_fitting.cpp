#include <Eigen/Core>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"

G2O_USE_OPTIMIZATION_LIBRARY(dense);

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

class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void setToOriginImpl() override { _estimate.setZero(); }

    void oplusImpl(const double* update) override {
        _estimate += Eigen::Vector3d(update);
    }

    bool read(std::istream& in) override { return false; }
    bool write(std::ostream& out) const override { return false; }
};

class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, CurveFittingVertex> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    template <typename T>
    bool operator()(const T* params, T* residual) const {
        const T& a = params[0];
        const T& b = params[1];
        const T& c = params[2];
        T y =
            a * measurement()(0) * measurement()(0) + b * measurement()(0) + c;
        residual[0] = y - measurement()(1);
        return true;
    }

    bool read(std::istream& in) override { return false; }
    bool write(std::ostream& out) const override { return false; }

    G2O_MAKE_AUTO_AD_FUNCTIONS
};

int main(int argc, char** argv) {
    // generate data
    const int num_data = 100;
    std::vector<Eigen::Vector2d> data = GenerateData(num_data);

    // set optimizer
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmProperty optimizer_property;
    optimizer.setAlgorithm(
        g2o::OptimizationAlgorithmFactory::instance()->construct(
            "lm_dense", optimizer_property));

    // add vertices
    CurveFittingVertex* params = new CurveFittingVertex();
    params->setId(0);
    params->setEstimate(Eigen::Vector3d(0., 0., 0.));
    optimizer.addVertex(params);

    // add edges
    for (int i = 0; i < num_data; ++i) {
        CurveFittingEdge* e = new CurveFittingEdge;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->setVertex(0, params);
        e->setMeasurement(data.at(i));
        optimizer.addEdge(e);
    }
    
    // optimization
    const int max_iterations = 10;
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(max_iterations);

    Eigen::Vector3d estmation = params->estimate();
    std::cout << "Initial a: " << 0.0 << ", b: " << 0.0 << ", c: " << 0.0
              << std::endl;
    std::cout << "Final   a: " << estmation[0] << ", b: " << estmation[1]
              << ", c: " << estmation[2] << std::endl;

    return 0;
}