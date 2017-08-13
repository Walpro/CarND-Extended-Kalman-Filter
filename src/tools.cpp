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
	/*initilising the rmse vector*/
	VectorXd rmse(4);
	/* Initialising the rmse vector */
	for(unsigned int i=0; i < rmse.size(); ++i){
		rmse[i] = 0;
	}



	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals

	cout<<"size of estimations array : " <<estimations.size()<<endl;
	for(unsigned int i=0; i < estimations.size(); ++i){
		VectorXd residual = estimations[i] - ground_truth[i];
		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

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
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	/*variables to simplify the computation*/
	float der1 = 0, der3 = 0;

	//TODO: YOUR CODE HERE

	//check division by zero
	if( (px == 0) && (py == 0) ){
	    cout << "Invalid devision by zero" << endl;
	}
	else
	{
	    // simplifying the denominator calculation for the first and second rows */
	    der1 = px*px + py*py;
	    // Computing the denominator for the third row*/
	    der3 = sqrt(der1)*sqrt(der1)*sqrt(der1);
	    // Calculating the Jacobin Matrix*/
	    Hj << px/sqrt(der1)         , py/sqrt(der1)         , 0          , 0         ,
	          -py/der1             , px/der1               , 0          , 0         ,
	          py*(vx*py-vy*px)/der3, px*(-vx*py+vy*px)/der3, px/sqrt(der1), py/sqrt(der1);

	    }

	return Hj;
}
