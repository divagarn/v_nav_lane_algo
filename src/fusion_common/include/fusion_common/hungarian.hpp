/**
 * @file hungarian.hpp
 * @brief Rectangular linear assignment (Hungarian / Kuhn-Munkres, O(n^3)).
 */
#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace fusion_common
{

/**
 * @brief Minimum-cost one-to-one assignment of rows to columns.
 *
 * The matrix is padded to square with a large cost; padded or gated pairs are not returned.
 * @param[in] cost  rows x cols cost matrix
 * @param[in] gate  entries >= gate are forbidden
 * @return assignment[row] = col, or -1 if the row is unassigned
 */
inline std::vector<int> hungarian(const Eigen::MatrixXd & cost, double gate)
{
  const int nr = static_cast<int>(cost.rows());
  const int nc = static_cast<int>(cost.cols());
  std::vector<int> result(nr, -1);
  if (nr == 0 || nc == 0) {return result;}
  const int n = std::max(nr, nc);
  const double big = gate * 10.0 + 1.0;
  Eigen::MatrixXd a = Eigen::MatrixXd::Constant(n, n, big);
  for (int i = 0; i < nr; ++i) {
    for (int j = 0; j < nc; ++j) {a(i, j) = cost(i, j) < gate ? cost(i, j) : big;}
  }
  // e-maxx implementation (1-indexed potentials u, v): reduced cost = a(i,j) - u_i - v_j >= 0
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0), v(n + 1, 0), minv(n + 1);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);
  std::vector<char> used(n + 1);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::fill(minv.begin(), minv.end(), INF);
    std::fill(used.begin(), used.end(), 0);
    do {
      used[j0] = 1;
      const int i0 = p[j0];
      double delta = INF;
      int j1 = 0;
      for (int j = 1; j <= n; ++j) {
        if (used[j]) {continue;}
        const double cur = a(i0 - 1, j - 1) - u[i0] - v[j];
        if (cur < minv[j]) {minv[j] = cur; way[j] = j0;}
        if (minv[j] < delta) {delta = minv[j]; j1 = j;}
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {u[p[j]] += delta; v[j] -= delta;} else {minv[j] -= delta;}
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }
  for (int j = 1; j <= n; ++j) {
    const int i = p[j] - 1;
    const int c = j - 1;
    if (i < nr && c < nc && cost(i, c) < gate) {result[i] = c;}
  }
  return result;
}

}  // namespace fusion_common
