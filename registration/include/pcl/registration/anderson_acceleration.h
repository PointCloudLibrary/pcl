#pragma once

#include <pcl/exceptions.h>
#include <pcl/pcl_macros.h>

#include <Eigen/Core>
#include <Eigen/QR>

#include <algorithm>
#include <cstddef>

namespace pcl {
namespace registration {

/**
 * \brief Lightweight Anderson acceleration helper used to speed up fixed-point
 *        iterations in FRICP.
 *
 * The class stores a short history of the most recent residuals and solves the
 * normal equations following the scheme described in "Fast and Robust Iterative
 * Closest Point", Zhang et al., 2022.
 *
 * Only double precision is supported internally to maximise numerical stability.
 */
class AndersonAcceleration {
public:
  AndersonAcceleration() = default;

  /**
   * \brief Initialise the accelerator with a circular buffer of size \a history.
   *
   * \param[in] history    Number of previous iterates to keep (m in the paper).
   * \param[in] dimension  Dimensionality of the flattened state vector.
   * \param[in] initial_state Pointer to the initial state (expects
   *                          \a dimension doubles).
   */
  inline void
  init(std::size_t history, std::size_t dimension, const double* initial_state)
  {
    if ((history == 0) || (dimension == 0)) {
      PCL_THROW_EXCEPTION(pcl::BadArgumentException,
                          "AndersonAcceleration::init expects non-zero sizes");
    }

    history_length_ = history;
    dimension_ = dimension;
    iteration_ = 0;
    column_index_ = 0;
    initialized_ = true;

    current_u_.resize(dimension_);
    current_F_.resize(dimension_);
    prev_dG_.setZero(dimension_, history_length_);
    prev_dF_.setZero(dimension_, history_length_);
    normal_eq_matrix_.setZero(history_length_, history_length_);
    theta_.setZero(history_length_);
    dF_scale_.setZero(history_length_);

    current_u_ = Eigen::Map<const Eigen::VectorXd>(initial_state, dimension_);
  }

  inline bool
  isInitialized() const
  {
    return initialized_;
  }

  inline std::size_t
  history() const
  {
    return history_length_;
  }

  inline std::size_t
  dimension() const
  {
    return dimension_;
  }

  inline void
  replace(const double* state)
  {
    if (!initialized_)
      return;
    current_u_ = Eigen::Map<const Eigen::VectorXd>(state, dimension_);
  }

  inline void
  reset(const double* state)
  {
    if (!initialized_)
      return;
    iteration_ = 0;
    column_index_ = 0;
    current_u_ = Eigen::Map<const Eigen::VectorXd>(state, dimension_);
  }

  /**
   * \brief Apply one Anderson acceleration update.
   *
   * \param[in] next_state Flattened state obtained from the fixed-point iteration.
   * \return Reference to the accelerated state vector.
   */
  inline const Eigen::VectorXd&
  compute(const double* next_state)
  {
    if (!initialized_) {
      PCL_THROW_EXCEPTION(pcl::PCLException,
                          "AndersonAcceleration::compute called before init");
    }

    Eigen::Map<const Eigen::VectorXd> G(next_state, dimension_);
    current_F_ = G - current_u_;

    constexpr double eps = 1e-14;

    if (iteration_ == 0) {
      prev_dF_.col(0) = -current_F_;
      prev_dG_.col(0) = -G;
      current_u_ = G;
    }
    else {
      prev_dF_.col(column_index_) += current_F_;
      prev_dG_.col(column_index_) += G;

      double scale = std::max(eps, prev_dF_.col(column_index_).norm());
      dF_scale_(column_index_) = scale;
      prev_dF_.col(column_index_) /= scale;

      const std::size_t m_k = std::min(history_length_, iteration_);

      if (m_k == 1) {
        theta_(0) = 0.0;
        const double dF_norm = prev_dF_.col(column_index_).norm();
        normal_eq_matrix_(0, 0) = dF_norm * dF_norm;
        if (dF_norm > eps) {
          theta_(0) = (prev_dF_.col(column_index_) / dF_norm).dot(current_F_ / dF_norm);
        }
      }
      else {
        // update Gram matrix row/column corresponding to the newest column
        const Eigen::VectorXd new_inner_prod = prev_dF_.col(column_index_).transpose() *
                                               prev_dF_.block(0, 0, dimension_, m_k);
        normal_eq_matrix_.block(column_index_, 0, 1, m_k) = new_inner_prod.transpose();
        normal_eq_matrix_.block(0, column_index_, m_k, 1) = new_inner_prod;

        cod_.compute(normal_eq_matrix_.block(0, 0, m_k, m_k));
        theta_.head(m_k) =
            cod_.solve(prev_dF_.block(0, 0, dimension_, m_k).transpose() * current_F_);
      }

      const Eigen::ArrayXd scaled_theta =
          theta_.head(m_k).array() / dF_scale_.head(m_k).array();
      current_u_ = G - prev_dG_.block(0, 0, dimension_, m_k) * scaled_theta.matrix();

      column_index_ = (column_index_ + 1) % history_length_;
      prev_dF_.col(column_index_) = -current_F_;
      prev_dG_.col(column_index_) = -G;
    }

    ++iteration_;
    return current_u_;
  }

private:
  std::size_t history_length_{0};
  std::size_t dimension_{0};
  std::size_t iteration_{0};
  std::size_t column_index_{0};
  bool initialized_{false};

  Eigen::VectorXd current_u_;
  Eigen::VectorXd current_F_;
  Eigen::MatrixXd prev_dG_;
  Eigen::MatrixXd prev_dF_;
  Eigen::MatrixXd normal_eq_matrix_;
  Eigen::VectorXd theta_;
  Eigen::VectorXd dF_scale_;
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace registration
} // namespace pcl
