#pragma once
#include <vector>
#include <Eigen/Dense>
#include <mass_properties.hpp>
using namespace mahi::util;

inline Eigen::VectorXd get_G(const std::vector<double>& qs){
	Eigen::VectorXd G = Eigen::VectorXd::Zero(4); 

	const double q0 = qs[0];
	const double q1 = qs[1];
	const double q2 = qs[2];
	const double q3 = qs[3];
	const double qd0 = qs[4];
	const double qd1 = qs[5];
	const double qd2 = qs[6];
	const double qd3 = qs[7];

	const double t2 = cos(q0);
	const double t3 = cos(q1);
	const double t4 = cos(q2);
	const double t5 = cos(q3);
	const double t6 = sin(q0);
	const double t7 = sin(q1);
	const double t8 = sin(q2);
	const double t9 = sin(q3);
	G(0,0) = g*(m1*t2*2.4269798E-1+m2*t2*2.4269798E-1+m3*t2*2.4269798E-1-Pcx0*m0*t6-Pcy0*m0*t2-Pcz1*m1*t2-Pcx2*m2*t2*t4+Pcx1*m1*t6*t7+Pcy1*m1*t3*t6+Pcy2*m2*t2*t8-Pcz2*m2*t3*t6+Pcz3*m3*t2*t8-Pcx3*m3*t2*t4*t5+Pcx3*m3*t3*t6*t9-Pcx2*m2*t6*t7*t8+Pcy3*m3*t3*t5*t6-Pcy2*m2*t4*t6*t7+Pcy3*m3*t2*t4*t9-Pcz3*m3*t4*t6*t7-Pcx3*m3*t5*t6*t7*t8+Pcy3*m3*t6*t7*t8*t9);
	G(1,0) = g*t2*(-Pcx1*m1*t3+Pcy1*m1*t7-Pcz2*m2*t7+Pcx2*m2*t3*t8+Pcx3*m3*t7*t9+Pcy2*m2*t3*t4+Pcy3*m3*t5*t7+Pcz3*m3*t3*t4+Pcx3*m3*t3*t5*t8-Pcy3*m3*t3*t8*t9);
	G(2,0) = g*(Pcx2*m2*t6*t8+Pcy2*m2*t4*t6+Pcz3*m3*t4*t6+Pcx2*m2*t2*t4*t7+Pcx3*m3*t5*t6*t8-Pcy2*m2*t2*t7*t8-Pcy3*m3*t6*t8*t9-Pcz3*m3*t2*t7*t8+Pcx3*m3*t2*t4*t5*t7-Pcy3*m3*t2*t4*t7*t9);
	G(3,0) = -g*m3*(Pcx3*t2*t3*t5-Pcx3*t4*t6*t9-Pcy3*t2*t3*t9-Pcy3*t4*t5*t6+Pcx3*t2*t7*t8*t9+Pcy3*t2*t5*t7*t8);

	return G;
}