#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (!estimations.size() || estimations.size() != ground_truth.size()) {
    cout<<"Estimation vector size should not be zero, or the should equal to ground truth!!!"<<endl;
    return rmse;
  }

  //accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) {
    VectorXd residuals = estimations[i] - ground_truth[i];
    VectorXd squared_residuals = residuals.array() * residuals.array();
    rmse = rmse + squared_residuals;
	}
	//calculate the mean
  rmse = rmse / estimations.size();
	//calculate the squared root
	rmse = rmse.array().sqrt();
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;
  // Check division by Zero
  if (fabs(c1) < 0.0001) {
    cout<<__FUNCTION__<<" (): Error Division by Zero!!!"<<endl;
    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;
    return Hj;
  }

  // Compute the Jacobian Matrix
  Hj << px / c2, py / c2, 0, 0,
       -py / c1, px / c1, 0, 0,
        py * (vx*py - vy*px) / c3, px * (vy*px - vx*py) / c3, px / c2, py / c2;
  return Hj;
}
