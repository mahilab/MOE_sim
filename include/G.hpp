#pragma once
#include <Mahi/Util.hpp>
#include <Eigen/Dense>
using namespace mahi::util;

inline Eigen::VectorXd get_G(const std::vector<double>& qs){
	Eigen::VectorXd G = Eigen::VectorXd::Zero(14); 

	double qe = qs[0];
	double qf = qs[1];
	double l1 = qs[2];
	double l2 = qs[3];
	double l3 = qs[4];
	double theta1 = qs[5];
	double theta2 = qs[6];
	double theta3 = qs[7];
	double P_p_x = qs[8];
	double P_p_y = qs[9];
	double P_p_z = qs[10];
	double R_p_x = qs[11];
	double R_p_y = qs[12];
	double R_p_z = qs[13];
	double sinqe = sin(qe);
	double cosqe = cos(qe);
	double sinqf = sin(qf);
	double cosqf = cos(qf);
	double sintheta1 = sin(theta1);
	double costheta1 = cos(theta1);
	double sintheta2 = sin(theta2);
	double costheta2 = cos(theta2);
	double sintheta3 = sin(theta3);
	double costheta3 = cos(theta3);
	double sinR_p_x = sin(R_p_x);
	double cosR_p_x = cos(R_p_x);
	double sinR_p_y = sin(R_p_y);
	double cosR_p_y = cos(R_p_y);
	double sinR_p_z = sin(R_p_z);
	double cosR_p_z = cos(R_p_z);

	G(0,0) = cosqe*-2.15587042410516+sinqe*6.502770032861918E-1+P_p_x*cosqe*3.579289451100067+sinqe*(cosqf*5.795003273562997E-1+sinqf*8.149720060185928E-1)*1.519201330992814E-1-sinqe*(cosqf*8.149720060185928E-1-sinqf*5.795003273562997E-1)*5.574031732461515E-4+(l2-8.803883999999584E-2)*(cosqe*sintheta2-costheta2*sinqe*(cosqf*4.160362969070661E-1-sinqf*9.093480080013023E-1))*1.45384239240002+(l1-8.803883999999584E-2)*(cosqe*sintheta1+costheta1*sinqe*(cosqf*9.955366242634227E-1-sinqf*9.437600198272378E-2))*1.45384239240002+(l3-8.803883999999584E-2)*(cosqe*sintheta3-costheta3*sinqe*(cosqf*5.795003273562997E-1+sinqf*8.149720060185928E-1)*1.0)*1.45384239240002+sinqe*(cosqf*4.160362969070661E-1-sinqf*9.093480080013023E-1)*1.519201330992814E-1+sinqe*(cosqf*9.093480080013023E-1+sinqf*4.160362969070661E-1)*5.574031732461515E-4-sinqe*(cosqf*9.437600198272378E-2+sinqf*9.955366242634227E-1)*5.574031732461515E-4-sinqe*(cosqf*9.955366242634227E-1-sinqf*9.437600198272378E-2)*1.519201330992814E-1-P_p_y*cosqf*sinqe*3.579289451100067+P_p_z*sinqe*sinqf*3.579289451100067;
	G(1,0) = cosqe*(cosqf*5.795003273562997E-1+sinqf*8.149720060185928E-1)*-5.574031732461515E-4-cosqe*(cosqf*8.149720060185928E-1-sinqf*5.795003273562997E-1)*1.519201330992814E-1-cosqe*(cosqf*4.160362969070661E-1-sinqf*9.093480080013023E-1)*5.574031732461515E-4+cosqe*(cosqf*9.093480080013023E-1+sinqf*4.160362969070661E-1)*1.519201330992814E-1-cosqe*(cosqf*9.437600198272378E-2+sinqf*9.955366242634227E-1)*1.519201330992814E-1+cosqe*(cosqf*9.955366242634227E-1-sinqf*9.437600198272378E-2)*5.574031732461515E-4-P_p_z*cosqe*cosqf*3.579289451100067-P_p_y*cosqe*sinqf*3.579289451100067+cosqe*costheta3*(l3-8.803883999999584E-2)*(cosqf*8.149720060185928E-1-sinqf*5.795003273562997E-1)*1.45384239240002-cosqe*costheta2*(l2-8.803883999999584E-2)*(cosqf*9.093480080013023E-1+sinqf*4.160362969070661E-1)*1.45384239240002+cosqe*costheta1*(l1-8.803883999999584E-2)*(cosqf*9.437600198272378E-2+sinqf*9.955366242634227E-1)*1.45384239240002;
	G(2,0) = sinqe*sintheta1*1.45384239240002-cosqe*costheta1*(cosqf*9.955366242634227E-1-sinqf*9.437600198272378E-2)*1.45384239240002;
	G(3,0) = sinqe*sintheta2*1.45384239240002+cosqe*costheta2*(cosqf*4.160362969070661E-1-sinqf*9.093480080013023E-1)*1.45384239240002;
	G(4,0) = sinqe*sintheta3*1.45384239240002+cosqe*costheta3*(cosqf*5.795003273562997E-1+sinqf*8.149720060185928E-1)*1.45384239240002;
	G(5,0) = (l1-8.803883999999584E-2)*(costheta1*sinqe+cosqe*sintheta1*(cosqf*9.955366242634227E-1-sinqf*9.437600198272378E-2))*1.45384239240002;
	G(6,0) = (l2-8.803883999999584E-2)*(costheta2*sinqe-cosqe*sintheta2*(cosqf*4.160362969070661E-1-sinqf*9.093480080013023E-1))*1.45384239240002;
	G(7,0) = (l3-8.803883999999584E-2)*(costheta3*sinqe-cosqe*sintheta3*(cosqf*5.795003273562997E-1+sinqf*8.149720060185928E-1)*1.0)*1.45384239240002;
	G(8,0) = sinqe*3.579289451100067;
	G(9,0) = cosqe*cosqf*3.579289451100067;
	G(10,0) = cosqe*sinqf*-3.579289451100067;
	return G;
}