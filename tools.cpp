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
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 

	//check division by zero
	if(0 == px && 0 == py)
	{
	    cout<<"Error dividing by zero!"<<endl;
	    return Hj;
	}
	//compute the Jacobian matrix
	float x2y2 = px*px+py*py;
	Hj << px/pow(x2y2,0.5) , py/pow(x2y2,0.5) , 0 , 0 ,
	      -py/x2y2, px/x2y2 , 0 , 0 ,
	      py*(vx*py - vy*px)/pow(x2y2, 1.5) , px*(vy*px - vx*py)/pow(x2y2, 1.5) , px/pow(x2y2 , 0.5), py/pow(x2y2 , 0.5);
	return Hj;
}
