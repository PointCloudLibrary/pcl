#include <iostream>
#include <stdexcept>

#include <pcl/surface/on_nurbs/nurbs_solve.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <chrono>
#include <vector>

using namespace pcl;
using namespace on_nurbs;

void
NurbsSolve::assign(unsigned rows, unsigned cols, unsigned dims)
{
  m_Ksparse.clear();
  m_xeig = Eigen::MatrixXd::Zero(cols, dims);
  m_feig = Eigen::MatrixXd::Zero(rows, dims);
}

void
NurbsSolve::K(unsigned i, unsigned j, double v)
{
  m_Ksparse.set(i, j, v);
}

void
NurbsSolve::x(unsigned i, unsigned j, double v)
{
  m_xeig(i, j) = v;
}

void
NurbsSolve::f(unsigned i, unsigned j, double v)
{
  m_feig(i, j) = v;
}

double
NurbsSolve::K(unsigned i, unsigned j)
{
  return m_Ksparse.get(i, j);
}

double
NurbsSolve::x(unsigned i, unsigned j)
{
  return m_xeig(i, j);
}

double
NurbsSolve::f(unsigned i, unsigned j)
{
  return m_feig(i, j);
}

void
NurbsSolve::resize(unsigned rows)
{
  m_feig.conservativeResize(rows, m_feig.cols());
  // Note: m_Ksparse is not resized here; assumed handled by assign()
}

void
NurbsSolve::printK()
{
  m_Ksparse.printLong();
}

void
NurbsSolve::printX()
{
  std::cout << m_xeig << std::endl;
}

void
NurbsSolve::printF()
{
  std::cout << m_feig << std::endl;
}

bool
NurbsSolve::solve()
{
  auto start_time = std::chrono::high_resolution_clock::now();
  if (!m_quiet) {
    std::cout << "[NurbsSolve] Start solving..." << std::endl;
  }

  int n_rows, n_cols;
  m_Ksparse.size(n_rows, n_cols);

  unsigned rows, cols, dims;
  getSize(rows, cols, dims);

  if (n_rows <= 0 || n_cols <= 0) {
    if (!m_quiet)
      std::cerr << "[NurbsSolve::solve] Invalid matrix size." << std::endl;
    return false;
  }

  // Convert SparseMat to Eigen::SparseMatrix
  std::vector<int> rowinds, colinds;
  std::vector<double> values;
  m_Ksparse.get(rowinds, colinds, values);

  // Use triplet list for efficient construction
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(values.size());
  for (size_t k = 0; k < values.size(); ++k) {
    tripletList.emplace_back(rowinds[k], colinds[k], values[k]);
  }

  Eigen::SparseMatrix<double> Keig_sparse(n_rows, n_cols);
  Keig_sparse.setFromTriplets(tripletList.begin(), tripletList.end());
  Keig_sparse.makeCompressed();

  // Choose solver
//    std::string solver_type = "Eigen::SparseQR";
//    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;


//  // NOTE: SparseLU may get wrong result in windows
//  std::string solver_type = "Eigen::SparseLU COLAMDOrdering";
//  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
//  std::string solver_type = "Eigen::SparseLU AMDOrdering";
//  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::AMDOrdering<int>> solver;

  std::string solver_type = "Eigen::SimplicialLDLT";
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;

  if (!m_quiet) {
    std::cout << "[NurbsSolve::solve] solver_type: " << solver_type << std::endl;
  }

  Eigen::SparseMatrix<double> KtK;
  Eigen::MatrixXd Ktf;
  Eigen::SparseMatrix<double> Kt;

  // For least-squares: solve min ||K * x - f||^2
  // We solve normal equations: (K^T K) x = K^T f
  Kt = Keig_sparse.transpose();
  KtK = Kt * Keig_sparse;
  Ktf = Kt * m_feig;

  // Solve KtK * x = Ktf
  solver.compute(KtK);
  if (solver.info() != Eigen::Success) {
    if (!m_quiet) {
      std::cerr << "[NurbsSolve::solve] compute failed" << std::endl;
      std::cerr << "[NurbsSolve::solve] solver.info: " << solver.info() << std::endl;
    }
    return false;
  }

  m_xeig = solver.solve(Ktf);
  if (solver.info() != Eigen::Success) {
    if (!m_quiet) {
      std::cerr << "[NurbsSolve::solve] Solve failed" << std::endl;
      std::cerr << "[NurbsSolve::solve] solver.info: " << solver.info() << std::endl;
    }
    return false;
  }

  if (!m_quiet) {
    auto end_time = std::chrono::high_resolution_clock::now(); // Record the end time
    auto duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time)
            .count();
    auto elapsed_time = static_cast<double>(duration) / 1000000000.0;
    std::cout << "[NurbsSolve] Solving completed. Time elapsed: " << elapsed_time
              << " seconds" << std::endl;
  }

  return true;
}
/*
Eigen::MatrixXd
NurbsSolve::diff ()
{
  int n_rows, n_cols;
  m_Ksparse.size (n_rows, n_cols);

  // Convert to Eigen sparse for multiplication
  std::vector<int> rowinds, colinds;
  std::vector<double> values;
  m_Ksparse.get (rowinds, colinds, values);

  std::vector<Eigen::Triplet<double>> tripletList;
  for (size_t k = 0; k < values.size(); ++k)
  {
    tripletList.emplace_back(rowinds[k], colinds[k], values[k]);
  }

  Eigen::SparseMatrix<double> Keig_sparse(n_rows, n_cols);
  Keig_sparse.setFromTriplets(tripletList.begin(), tripletList.end());

  Eigen::MatrixXd Kx = Keig_sparse * m_xeig;
  return Kx - m_feig;
}*/
Eigen::MatrixXd
NurbsSolve::diff()
{

  int n_rows, n_cols, n_dims;
  m_Ksparse.size(n_rows, n_cols);
  n_dims = m_feig.cols();

  if (n_rows != m_feig.rows()) {
    printf("[NurbsSolve::diff] K.rows: %d  f.rows: %d\n", n_rows, static_cast<int>(m_feig.rows()));
    throw std::runtime_error("[NurbsSolve::diff] Rows of equation do not match\n");
  }

  Eigen::MatrixXd f = Eigen::MatrixXd::Zero(n_rows, n_dims);

  for (int r = 0; r < n_rows; r++) {
    for (int c = 0; c < n_cols; c++) {
      f.row(r) = f.row(r) + m_xeig.row(c) * m_Ksparse.get(r, c);
    }
  }

  return (f - m_feig);
}
