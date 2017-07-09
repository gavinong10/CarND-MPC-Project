#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

extern const double Lf;

extern size_t x_start;
extern size_t y_start;
extern size_t psi_start;
extern size_t v_start;
extern size_t cte_start;
extern size_t epsi_start;
extern size_t delta_start;
extern size_t a_start;

extern size_t N;

#endif /* MPC_H */
