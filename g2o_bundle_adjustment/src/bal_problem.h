// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//

#ifndef __BAL_PROBLEM_H__
#define __BAL_PROBLEM_H__

#include <ceres/rotation.h>

#include <fstream>

// Read a Bundle Adjustment in the Large dataset.
class BALProblem {
  public:
    ~BALProblem() {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    int num_observations() const { return num_observations_; }
    int num_cameras() const { return num_cameras_; }
    int num_points() const { return num_points_; }
    int camera_block_size() const { return 9; }
    int point_block_size() const { return 3; }
    const int* point_index() const { return point_index_; }
    const int* camera_index() const { return camera_index_; }
    const double* observations() const { return observations_; }
    double* mutable_cameras() { return parameters_; }
    double* mutable_points() { return parameters_ + 9 * num_cameras_; }

    double* mutable_camera_for_observation(int i) {
        return mutable_cameras() + camera_index_[i] * 9;
    }
    double* mutable_point_for_observation(int i) {
        return mutable_points() + point_index_[i] * 3;
    }

    bool LoadFile(const char* filename) {
        FILE* fptr = fopen(filename, "r");
        if (fptr == nullptr) {
            return false;
        };

        FscanfOrDie(fptr, "%d", &num_cameras_);
        FscanfOrDie(fptr, "%d", &num_points_);
        FscanfOrDie(fptr, "%d", &num_observations_);

        point_index_ = new int[num_observations_];
        camera_index_ = new int[num_observations_];
        observations_ = new double[2 * num_observations_];

        num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
        parameters_ = new double[num_parameters_];

        for (int i = 0; i < num_observations_; ++i) {
            FscanfOrDie(fptr, "%d", camera_index_ + i);
            FscanfOrDie(fptr, "%d", point_index_ + i);
            for (int j = 0; j < 2; ++j) {
                FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
            }
        }

        for (int i = 0; i < num_parameters_; ++i) {
            FscanfOrDie(fptr, "%lf", parameters_ + i);
        }
        return true;
    }

    void WriteToPLYFile(const std::string& filename) const {
        std::ofstream of(filename.c_str());

        of << "ply" << '\n'
           << "format ascii 1.0" << '\n'
           << "element vertex " << num_cameras_ + num_points_ << '\n'
           << "property float x" << '\n'
           << "property float y" << '\n'
           << "property float z" << '\n'
           << "property uchar red" << '\n'
           << "property uchar green" << '\n'
           << "property uchar blue" << '\n'
           << "end_header" << std::endl;

        // Export extrinsic data (i.e. camera centers) as green points.
        double angle_axis[3];
        double center[3];
        for (int i = 0; i < num_cameras_; ++i) {
            const double* camera = parameters_ + 9 * i;
            CameraToAngleAxisAndCenter(camera, angle_axis, center);
            of << center[0] << ' ' << center[1] << ' ' << center[2]
               << " 0 255 0" << '\n';
        }

        // Export the structure (i.e. 3D Points) as white points.
        const double* points = parameters_ + 9 * num_cameras_;
        for (int i = 0; i < num_points_; ++i) {
            const double* point = points + i * 3;
            for (int j = 0; j < 3; ++j) {
                of << point[j] << ' ';
            }
            of << "255 255 255\n";
        }
        of.close();
    }

    void CameraToAngleAxisAndCenter(const double* camera, double* angle_axis,
                                    double* center) const {
        Eigen::Map<Eigen::VectorXd> angle_axis_ref(angle_axis, 3);
        angle_axis_ref = Eigen::Map<const Eigen::VectorXd>(camera, 3);

        // c = -R't
        Eigen::VectorXd inverse_rotation = -angle_axis_ref;
        ceres::AngleAxisRotatePoint(inverse_rotation.data(), camera + 9 - 6,
                                    center);
        Eigen::Map<Eigen::VectorXd>(center, 3) *= -1.0;
    }

  private:
    template <typename T>
    void FscanfOrDie(FILE* fptr, const char* format, T* value) {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1) {
            LOG(FATAL) << "Invalid UW data file.";
        }
    }

    int num_cameras_;
    int num_points_;
    int num_observations_;
    int num_parameters_;

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;
};

#endif