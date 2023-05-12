#include <Eigen/Core>
#include <iostream>

#include "bal_problem.h"
#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"

G2O_USE_OPTIMIZATION_LIBRARY(dense);

/* camera vertex which stores the parameters for a pinhole camera */
class CameraVertex : public g2o::BaseVertex<9, g2o::VectorN<9>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void setToOriginImpl() override {}

    void oplusImpl(const double* update) override {
        _estimate += g2o::VectorN<9>(update);
    }

    bool read(std::istream& in) override { return false; }
    bool write(std::ostream& out) const override { return false; }
};

/* point vertex which stores coordinates of 3d world feature */
class PointVertex : public g2o::BaseVertex<3, g2o::Vector3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void setToOriginImpl() override {}

    void oplusImpl(const double* update) override {
        _estimate += g2o::VectorN<3>(update);
    }

    bool read(std::istream& in) override { return false; }
    bool write(std::ostream& out) const override { return false; }
};

/* edge representing the observation of a world feature by a camera */
class EdgeObservationBAL
    : public g2o::BaseBinaryEdge<2, g2o::Vector2, CameraVertex, PointVertex> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    template <typename T>
    bool operator()(const T* p_camera, const T* p_point, T* p_error) const {
        typename g2o::VectorN<9, T>::ConstMapType camera(p_camera);
        typename g2o::VectorN<3, T>::ConstMapType point(p_point);

        typename g2o::VectorN<3, T> p;

        // Rodrigues' formula for the rotation
        T theta = camera.template head<3>().norm();
        if (theta > T(0)) {
            g2o::VectorN<3, T> v = camera.template head<3>() / theta;
            T cth = cos(theta);
            T sth = sin(theta);

            g2o::VectorN<3, T> vXp = v.cross(point);
            T vDotp = v.dot(point);
            T oneMinusCth = T(1) - cth;

            p = point * cth + vXp * sth + v * vDotp * oneMinusCth;
        } else {
            // taylor expansion for theta close to zero
            p = point + camera.template head<3>().cross(point);
        }

        // translation of the camera
        p += camera.template segment<3>(3);

        // perspective division
        g2o::VectorN<2, T> projectedPoint = -p.template head<2>() / p(2);

        // conversion to pixel coordinates
        T radiusSqr = projectedPoint.squaredNorm();
        const T& f = camera(6);
        const T& k1 = camera(7);
        const T& k2 = camera(8);
        T r_p = T(1) + k1 * radiusSqr + k2 * radiusSqr * radiusSqr;
        g2o::VectorN<2, T> prediction = f * r_p * projectedPoint;

        // compute the error
        typename g2o::VectorN<2, T>::MapType error(p_error);
        error = prediction - measurement().cast<T>();
        (void)error;

        return true;
    }

    bool read(std::istream& in) override { return false; }
    bool write(std::ostream& out) const override { return false; }

    G2O_MAKE_AUTO_AD_FUNCTIONS
};

int main(int argc, char** argv) {
    // load bal data
    BALProblem bal_problem;
    bal_problem.LoadFile("../data/problem-16-22106-pre.txt");
    bal_problem.WriteToPLYFile("before_ba.ply");

    // set optimizer
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmProperty optimizer_property;
    optimizer.setAlgorithm(
        g2o::OptimizationAlgorithmFactory::instance()->construct(
            "lm_dense", optimizer_property));

    // add camera_vertex, point_vertex and observation edge
    double* cameras = bal_problem.mutable_cameras();
    double* points = bal_problem.mutable_points();
    const int camera_block_size = bal_problem.camera_block_size();
    const int point_block_size = bal_problem.point_block_size();

    int id = 0;
    std::vector<CameraVertex*> camera_vertices;
    for (int i = 0; i < bal_problem.num_cameras(); ++i) {
        id += 1;
        CameraVertex* camera_vertex = new CameraVertex;
        camera_vertex->setId(id);
        double* camera = cameras + camera_block_size * i;
        camera_vertex->setEstimate(g2o::VectorN<9>(camera));
        optimizer.addVertex(camera_vertex);

        camera_vertices.push_back(camera_vertex);
    }

    std::vector<PointVertex*> point_vertices;
    for (int i = 0; i < bal_problem.num_points(); ++i) {
        id += 1;
        PointVertex* point_vertex = new PointVertex;
        point_vertex->setId(id);
        point_vertex->setMarginalized(true);
        double* point = points + point_block_size * i;
        point_vertex->setEstimate(Eigen::Vector3d(point));
        optimizer.addVertex(point_vertex);

        point_vertices.push_back(point_vertex);
    }

    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        EdgeObservationBAL* edge = new EdgeObservationBAL;
        edge->setVertex(0, camera_vertices[bal_problem.camera_index()[i]]);
        edge->setVertex(1, point_vertices[bal_problem.point_index()[i]]);

        const double* observations = bal_problem.observations();
        edge->setMeasurement(
            Eigen::Vector2d(observations[2 * i + 0], observations[2 * i + 1]));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
    }

    // optimization
    const int max_iterations = 25;
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(max_iterations);

    // set result to bal_problem
    for (int i = 0; i < bal_problem.num_cameras(); ++i) {
        double* camera = cameras + camera_block_size * i;
        auto vertex = camera_vertices[i];
        for (int j = 0; j < bal_problem.camera_block_size(); ++j) {
            camera[j] = vertex->estimate()[j];
        }
    }
    for (int i = 0; i < bal_problem.num_points(); ++i) {
        double* point = points + point_block_size * i;
        auto vertex = point_vertices[i];
        for (int j = 0; j < bal_problem.point_block_size(); ++j) {
            point[j] = vertex->estimate()[j];
        }
    }
    bal_problem.WriteToPLYFile("after_ba.ply");

    return 0;
}