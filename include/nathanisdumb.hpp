#pragma once
#include <vector>
#include <Eigen/Dense>
#include <mass_properties.hpp>
using namespace mahi::util;

inline double sq(double num){
	return num*num;
}

inline Eigen::MatrixXd get_nathanisdumb(const std::vector<double>& qs){
	Eigen::MatrixXd nathanisdumb = Eigen::MatrixXd::Zero(4,4); 

	const double q0 = qs[0];
	const double q1 = qs[1];
	const double q2 = qs[2];
	const double q3 = qs[3];
	const double qd0 = qs[4];
	const double qd1 = qs[5];
	const double qd2 = qs[6];
	const double qd3 = qs[7];
	const double sintheta0 = sin(q0);
	const double costheta0 = cos(q0);
	const double sintheta1 = sin(q1);
	const double costheta1 = cos(q1);
	const double sintheta2 = sin(q2);
	const double costheta2 = cos(q2);
	const double sintheta3 = sin(q3);
	const double costheta3 = cos(q3);

	const double var1=0.0225;
	const double var2=2;
	const double var3=0.3;
	const double var4=4;
	const double var5=((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((Icxx3+Icyy1)+Iczz0)+Iczz2)+(var1*m1))+(var1*m2))+(var1*m3))+(sq(Pcx0)*m0))+(sq(Pcx1)*m1))+(sq(Pcx2)*m2))+(sq(Pcy0)*m0))+(sq(Pcy2)*m2))+(sq(Pcy3)*m3))+(sq(Pcz1)*m1))+(sq(Pcz3)*m3))+(Icxx1*sq(costheta1)))-(Icxx3*sq(costheta1)))-(Icxx3*sq(costheta3)))-(Icyy1*sq(costheta1)))+(Icyy2*sq(costheta1)))+(Icyy3*sq(costheta3)))-(Iczz2*sq(costheta1)))+(Iczz3*sq(costheta1)))+(Icxy1*sin((var2*q1))))-(Icxy3*sin((var2*q3))))-(var3*((Pcx1*m1)*sintheta1)))+((Icxx2*sq(costheta1))*sq(costheta2)))+((Icxx3*sq(costheta1))*sq(costheta3)))-((Icyy2*sq(costheta1))*sq(costheta2)))+((Icyy3*sq(costheta1))*sq(costheta2)))-((Icyy3*sq(costheta1))*sq(costheta3)))-((Iczz3*sq(costheta1))*sq(costheta2)))-((sq(Pcx1)*m1)*sq(costheta1)))+((sq(Pcx3)*m3)*sq(costheta1)))+((sq(Pcx3)*m3)*sq(costheta3)))+((sq(Pcy1)*m1)*sq(costheta1)))-((sq(Pcy2)*m2)*sq(costheta1)))-((sq(Pcy3)*m3)*sq(costheta3)))+((sq(Pcz2)*m2)*sq(costheta1)))-((sq(Pcz3)*m3)*sq(costheta1)))-(var3*((Pcy1*m1)*costheta1)))+(var3*((Pcz2*m2)*costheta1)))+(var2*(((Icxy3*costheta1)*costheta2)*sintheta1)))-(var2*(((Icxz2*costheta1)*costheta2)*sintheta1)))+(((Icxx3*sq(costheta1))*sq(costheta2))*sq(costheta3)))-(((Icyy3*sq(costheta1))*sq(costheta2))*sq(costheta3)))+(var2*(((Icyz2*costheta1)*sintheta1)*sintheta2)))+(var2*(((Icxy2*sq(costheta1))*costheta2)*sintheta2)))+(var2*(((Icxy3*sq(costheta1))*costheta3)*sintheta3)))+(((Pcx1*Pcy1)*m1)*sin((var2*q1))))-(((Pcx3*Pcy3)*m3)*sin((var2*q3))))+(var3*(((Pcy3*m3)*costheta1)*costheta3)))-(var3*(((Pcx2*m2)*costheta2)*sintheta1)))+(var3*(((Pcx3*m3)*costheta1)*sintheta3)))-(((sq(Pcx2)*m2)*sq(costheta1))*sq(costheta2)))-(((sq(Pcx3)*m3)*sq(costheta1))*sq(costheta3)))+(((sq(Pcy2)*m2)*sq(costheta1))*sq(costheta2)))-(((sq(Pcy3)*m3)*sq(costheta1))*sq(costheta2)))+(((sq(Pcy3)*m3)*sq(costheta1))*sq(costheta3)))+(((sq(Pcz3)*m3)*sq(costheta1))*sq(costheta2)))+(var3*(((Pcy2*m2)*sintheta1)*sintheta2)))-(var3*(((Pcz3*m3)*sintheta1)*sintheta2)))-((((sq(Pcx3)*m3)*sq(costheta1))*sq(costheta2))*sq(costheta3)))+((((sq(Pcy3)*m3)*sq(costheta1))*sq(costheta2))*sq(costheta3)))-(var2*((((Icyz3*costheta1)*costheta3)*sintheta1)*sintheta2)))-(var2*((((Icxz3*costheta1)*sintheta1)*sintheta2)*sintheta3)))-(var4*((((Icxy3*costheta1)*costheta2)*sq(costheta3))*sintheta1)))-(var2*((((Icxz3*sq(costheta1))*costheta2)*costheta3)*sintheta2)))+(var2*((((Icyz3*sq(costheta1))*costheta2)*sintheta2)*sintheta3)))-(var3*((((Pcx3*m3)*costheta2)*costheta3)*sintheta1)))+(var3*((((Pcy3*m3)*costheta2)*sintheta1)*sintheta3)))+(var2*((((Icxy3*sq(costheta1))*sq(costheta2))*costheta3)*sintheta3)))+(var2*(((((Pcy2*Pcz2)*m2)*costheta1)*sintheta1)*sintheta2)))+(var2*(((((Pcx2*Pcy2)*m2)*sq(costheta1))*costheta2)*sintheta2)))+(var2*(((((Pcx3*Pcy3)*m3)*sq(costheta1))*costheta3)*sintheta3)))+(var2*(((((Icxx3*costheta1)*costheta2)*costheta3)*sintheta1)*sintheta3)))-(var2*(((((Icyy3*costheta1)*costheta2)*costheta3)*sintheta1)*sintheta3)))+(var2*(((((Pcx3*Pcy3)*m3)*costheta1)*costheta2)*sintheta1)))-(var2*(((((Pcx2*Pcz2)*m2)*costheta1)*costheta2)*sintheta1)))-(var4*((((((Pcx3*Pcy3)*m3)*costheta1)*costheta2)*sq(costheta3))*sintheta1)))-(var2*((((((Pcx3*Pcz3)*m3)*sq(costheta1))*costheta2)*costheta3)*sintheta2)))+(var2*((((((Pcy3*Pcz3)*m3)*sq(costheta1))*costheta2)*sintheta2)*sintheta3)))+(var2*((((((Pcx3*Pcy3)*m3)*sq(costheta1))*sq(costheta2))*costheta3)*sintheta3)))-(var2*((((((sq(Pcx3)*m3)*costheta1)*costheta2)*costheta3)*sintheta1)*sintheta3)))+(var2*((((((sq(Pcy3)*m3)*costheta1)*costheta2)*costheta3)*sintheta1)*sintheta3)))-(var2*((((((Pcy3*Pcz3)*m3)*costheta1)*costheta3)*sintheta1)*sintheta2)))-(var2*((((((Pcx3*Pcz3)*m3)*costheta1)*sintheta1)*sintheta2)*sintheta3)));
	const double var6=((((((((((((((((((((((((((((((((((((((((((((((Icxy2*costheta1)-(Icxz1*costheta1))+(Icyz1*sintheta1))-((Icxz3*costheta1)*costheta3))-((Icyz2*costheta2)*sintheta1))+((Icyz3*costheta1)*sintheta3))+((Icxy3*sintheta1)*sintheta2))-((Icxz2*sintheta1)*sintheta2))-(var2*((Icxy2*costheta1)*sq(costheta2))))+(((Icxx2*costheta1)*costheta2)*sintheta2))-(((Icyy2*costheta1)*costheta2)*sintheta2))+(((Icyy3*costheta1)*costheta2)*sintheta2))+(((Icyz3*costheta2)*costheta3)*sintheta1))-(((Iczz3*costheta1)*costheta2)*sintheta2))+(((Pcx2*Pcy2)*m2)*costheta1))-(((Pcx1*Pcz1)*m1)*costheta1))+(((Icxz3*costheta2)*sintheta1)*sintheta3))+(((Pcy1*Pcz1)*m1)*sintheta1))+(var2*(((Icxz3*costheta1)*sq(costheta2))*costheta3)))-(var2*(((Icyz3*costheta1)*sq(costheta2))*sintheta3)))-(var2*(((Icxy3*sq(costheta3))*sintheta1)*sintheta2)))-((((sq(Pcx2)*m2)*costheta1)*costheta2)*sintheta2))+((((sq(Pcy2)*m2)*costheta1)*costheta2)*sintheta2))-((((sq(Pcy3)*m3)*costheta1)*costheta2)*sintheta2))+((((sq(Pcz3)*m3)*costheta1)*costheta2)*sintheta2))-((((Pcx3*Pcz3)*m3)*costheta1)*costheta3))-((((Pcy2*Pcz2)*m2)*costheta2)*sintheta1))+((((Pcy3*Pcz3)*m3)*costheta1)*sintheta3))+((((Icxx3*costheta3)*sintheta1)*sintheta2)*sintheta3))-((((Icyy3*costheta3)*sintheta1)*sintheta2)*sintheta3))+((((Pcx3*Pcy3)*m3)*sintheta1)*sintheta2))-((((Pcx2*Pcz2)*m2)*sintheta1)*sintheta2))+((((Icxx3*costheta1)*costheta2)*sq(costheta3))*sintheta2))-((((Icyy3*costheta1)*costheta2)*sq(costheta3))*sintheta2))-(var2*((((Pcx2*Pcy2)*m2)*costheta1)*sq(costheta2))))+(((((Pcx3*Pcz3)*m3)*costheta2)*sintheta1)*sintheta3))-(((((sq(Pcx3)*m3)*costheta1)*costheta2)*sq(costheta3))*sintheta2))+(((((sq(Pcy3)*m3)*costheta1)*costheta2)*sq(costheta3))*sintheta2))+(var2*(((((Pcx3*Pcz3)*m3)*costheta1)*sq(costheta2))*costheta3)))-(var2*(((((Pcy3*Pcz3)*m3)*costheta1)*sq(costheta2))*sintheta3)))-(var2*(((((Pcx3*Pcy3)*m3)*sq(costheta3))*sintheta1)*sintheta2)))+(var2*(((((Icxy3*costheta1)*costheta2)*costheta3)*sintheta2)*sintheta3)))+(((((Pcy3*Pcz3)*m3)*costheta2)*costheta3)*sintheta1))-(((((sq(Pcx3)*m3)*costheta3)*sintheta1)*sintheta2)*sintheta3))+(((((sq(Pcy3)*m3)*costheta3)*sintheta1)*sintheta2)*sintheta3))+(var2*(((((((Pcx3*Pcy3)*m3)*costheta1)*costheta2)*costheta3)*sintheta2)*sintheta3)));
 	const double var7=0.15;
	const double var8=(((((((((((((((((((((((((((((((((Icxx3*sintheta1)+(Iczz2*sintheta1))+(var7*((Pcy2*m2)*sintheta2)))-(var7*((Pcz3*m3)*sintheta2)))+((Icxy3*costheta1)*costheta2))-((Icxz2*costheta1)*costheta2))+((sq(Pcx2)*m2)*sintheta1))+((sq(Pcy2)*m2)*sintheta1))+((sq(Pcy3)*m3)*sintheta1))+((sq(Pcz3)*m3)*sintheta1))+((Icyz2*costheta1)*sintheta2))-((Icxx3*sq(costheta3))*sintheta1))+((Icyy3*sq(costheta3))*sintheta1))-(var7*((Pcx2*m2)*costheta2)))-(((Icyz3*costheta1)*costheta3)*sintheta2))-(var2*(((Icxy3*costheta3)*sintheta1)*sintheta3)))-(((Icxz3*costheta1)*sintheta2)*sintheta3))-(var2*(((Icxy3*costheta1)*costheta2)*sq(costheta3))))+(((sq(Pcx3)*m3)*sq(costheta3))*sintheta1))-(((sq(Pcy3)*m3)*sq(costheta3))*sintheta1))-(var7*(((Pcx3*m3)*costheta2)*costheta3)))+(var7*(((Pcy3*m3)*costheta2)*sintheta3)))+((((Icxx3*costheta1)*costheta2)*costheta3)*sintheta3))-((((Icyy3*costheta1)*costheta2)*costheta3)*sintheta3))+((((Pcx3*Pcy3)*m3)*costheta1)*costheta2))-((((Pcx2*Pcz2)*m2)*costheta1)*costheta2))+((((Pcy2*Pcz2)*m2)*costheta1)*sintheta2))-(var2*(((((Pcx3*Pcy3)*m3)*costheta3)*sintheta1)*sintheta3)))-(((((Pcx3*Pcz3)*m3)*costheta1)*sintheta2)*sintheta3))-(var2*(((((Pcx3*Pcy3)*m3)*costheta1)*costheta2)*sq(costheta3))))-(((((sq(Pcx3)*m3)*costheta1)*costheta2)*costheta3)*sintheta3))+(((((sq(Pcy3)*m3)*costheta1)*costheta2)*costheta3)*sintheta3))-(((((Pcy3*Pcz3)*m3)*costheta1)*costheta3)*sintheta2));
	const double var9=((((((((((((((Iczz3*costheta1)*sintheta2)-((Icyz3*costheta3)*sintheta1))-((Icxz3*sintheta1)*sintheta3))-(((Icxz3*costheta1)*costheta2)*costheta3))+(((sq(Pcx3)*m3)*costheta1)*sintheta2))+(((sq(Pcy3)*m3)*costheta1)*sintheta2))+(((Icyz3*costheta1)*costheta2)*sintheta3))+(var7*(((Pcy3*m3)*costheta3)*sintheta2)))+(var7*(((Pcx3*m3)*sintheta2)*sintheta3)))-((((Pcy3*Pcz3)*m3)*costheta3)*sintheta1))-((((Pcx3*Pcz3)*m3)*sintheta1)*sintheta3))-(((((Pcx3*Pcz3)*m3)*costheta1)*costheta2)*costheta3))+(((((Pcy3*Pcz3)*m3)*costheta1)*costheta2)*sintheta3));
	const double var10=sqrt((((sq(var5)+sq(var6))+sq(var8))+sq(var9)));
	const double var11=(var5/var10);
	const double var12=((((((((((((((Iczz3*costheta1)*sintheta2)-((Icyz3*costheta3)*sintheta1))-((Icxz3*sintheta1)*sintheta3))-(((Icxz3*costheta1)*costheta2)*costheta3))+(((sq(Pcx3)*m3)*costheta1)*sintheta2))+(((sq(Pcy3)*m3)*costheta1)*sintheta2))+(((Icyz3*costheta1)*costheta2)*sintheta3))+(var7*(((Pcy3*m3)*costheta3)*sintheta2)))+(var7*(((Pcx3*m3)*sintheta2)*sintheta3)))-((((Pcy3*Pcz3)*m3)*costheta3)*sintheta1))-((((Pcx3*Pcz3)*m3)*sintheta1)*sintheta3))-(((((Pcx3*Pcz3)*m3)*costheta1)*costheta2)*costheta3))+(((((Pcy3*Pcz3)*m3)*costheta1)*costheta2)*sintheta3));
	const double var13=(((((Icyz3*sintheta2)*sintheta3)-((((Iczz3*costheta2)+((sq(Pcx3)*m3)*costheta2))+((sq(Pcy3)*m3)*costheta2))+((Icxz3*costheta3)*sintheta2)))-((((Pcx3*Pcz3)*m3)*costheta3)*sintheta2))+((((Pcy3*Pcz3)*m3)*sintheta2)*sintheta3));
	const double var14=(var6/var10);
	const double var15=((((Icyz3*costheta3)+(Icxz3*sintheta3))+(((Pcy3*Pcz3)*m3)*costheta3))+(((Pcx3*Pcz3)*m3)*sintheta3));
	const double var16=(var8/var10);
	const double var17=((Iczz3+(sq(Pcx3)*m3))+(sq(Pcy3)*m3));
	const double var18=(var9/var10);
	const double var19=((((var12*var11)+(var13*var14))-(var15*var16))+(var17*var18));
	const double var20=(var12-(var19*var11));
	const double var21=((((((((((((((((((((((((((((((((((((((((((((((Icxy2*costheta1)-(Icxz1*costheta1))+(Icyz1*sintheta1))-((Icxz3*costheta1)*costheta3))-((Icyz2*costheta2)*sintheta1))+((Icyz3*costheta1)*sintheta3))+((Icxy3*sintheta1)*sintheta2))-((Icxz2*sintheta1)*sintheta2))-(var2*((Icxy2*costheta1)*sq(costheta2))))+(((Icxx2*costheta1)*costheta2)*sintheta2))-(((Icyy2*costheta1)*costheta2)*sintheta2))+(((Icyy3*costheta1)*costheta2)*sintheta2))+(((Icyz3*costheta2)*costheta3)*sintheta1))-(((Iczz3*costheta1)*costheta2)*sintheta2))+(((Pcx2*Pcy2)*m2)*costheta1))-(((Pcx1*Pcz1)*m1)*costheta1))+(((Icxz3*costheta2)*sintheta1)*sintheta3))+(((Pcy1*Pcz1)*m1)*sintheta1))+(var2*(((Icxz3*costheta1)*sq(costheta2))*costheta3)))-(var2*(((Icyz3*costheta1)*sq(costheta2))*sintheta3)))-(var2*(((Icxy3*sq(costheta3))*sintheta1)*sintheta2)))-((((sq(Pcx2)*m2)*costheta1)*costheta2)*sintheta2))+((((sq(Pcy2)*m2)*costheta1)*costheta2)*sintheta2))-((((sq(Pcy3)*m3)*costheta1)*costheta2)*sintheta2))+((((sq(Pcz3)*m3)*costheta1)*costheta2)*sintheta2))-((((Pcx3*Pcz3)*m3)*costheta1)*costheta3))-((((Pcy2*Pcz2)*m2)*costheta2)*sintheta1))+((((Pcy3*Pcz3)*m3)*costheta1)*sintheta3))+((((Icxx3*costheta3)*sintheta1)*sintheta2)*sintheta3))-((((Icyy3*costheta3)*sintheta1)*sintheta2)*sintheta3))+((((Pcx3*Pcy3)*m3)*sintheta1)*sintheta2))-((((Pcx2*Pcz2)*m2)*sintheta1)*sintheta2))+((((Icxx3*costheta1)*costheta2)*sq(costheta3))*sintheta2))-((((Icyy3*costheta1)*costheta2)*sq(costheta3))*sintheta2))-(var2*((((Pcx2*Pcy2)*m2)*costheta1)*sq(costheta2))))+(((((Pcx3*Pcz3)*m3)*costheta2)*sintheta1)*sintheta3))-(((((sq(Pcx3)*m3)*costheta1)*costheta2)*sq(costheta3))*sintheta2))+(((((sq(Pcy3)*m3)*costheta1)*costheta2)*sq(costheta3))*sintheta2))+(var2*(((((Pcx3*Pcz3)*m3)*costheta1)*sq(costheta2))*costheta3)))-(var2*(((((Pcy3*Pcz3)*m3)*costheta1)*sq(costheta2))*sintheta3)))-(var2*(((((Pcx3*Pcy3)*m3)*sq(costheta3))*sintheta1)*sintheta2)))+(var2*(((((Icxy3*costheta1)*costheta2)*costheta3)*sintheta2)*sintheta3)))+(((((Pcy3*Pcz3)*m3)*costheta2)*costheta3)*sintheta1))-(((((sq(Pcx3)*m3)*costheta3)*sintheta1)*sintheta2)*sintheta3))+(((((sq(Pcy3)*m3)*costheta3)*sintheta1)*sintheta2)*sintheta3))+(var2*(((((((Pcx3*Pcy3)*m3)*costheta1)*costheta2)*costheta3)*sintheta2)*sintheta3)));
	const double var22=((((((((((((((((((((((((((((((((((Icxx2+Icyy3)+Iczz1)+(sq(Pcx1)*m1))+(sq(Pcx3)*m3))+(sq(Pcy1)*m1))+(sq(Pcy2)*m2))+(sq(Pcz2)*m2))+(sq(Pcz3)*m3))-(Icxx2*sq(costheta2)))+(Icxx3*sq(costheta3)))+(Icyy2*sq(costheta2)))-(Icyy3*sq(costheta2)))-(Icyy3*sq(costheta3)))+(Iczz3*sq(costheta2)))-(Icxy2*sin((var2*q2))))+(Icxy3*sin((var2*q3))))-((Icxx3*sq(costheta2))*sq(costheta3)))+((Icyy3*sq(costheta2))*sq(costheta3)))+((sq(Pcx2)*m2)*sq(costheta2)))-((sq(Pcx3)*m3)*sq(costheta3)))-((sq(Pcy2)*m2)*sq(costheta2)))+((sq(Pcy3)*m3)*sq(costheta2)))+((sq(Pcy3)*m3)*sq(costheta3)))-((sq(Pcz3)*m3)*sq(costheta2)))+(var2*(((Icxz3*costheta2)*costheta3)*sintheta2)))-(var2*(((Icyz3*costheta2)*sintheta2)*sintheta3)))-(var2*(((Icxy3*sq(costheta2))*costheta3)*sintheta3)))-(((Pcx2*Pcy2)*m2)*sin((var2*q2))))+(((Pcx3*Pcy3)*m3)*sin((var2*q3))))+(((sq(Pcx3)*m3)*sq(costheta2))*sq(costheta3)))-(((sq(Pcy3)*m3)*sq(costheta2))*sq(costheta3)))-(var2*(((((Pcy3*Pcz3)*m3)*costheta2)*sintheta2)*sintheta3)))-(var2*(((((Pcx3*Pcy3)*m3)*sq(costheta2))*costheta3)*sintheta3)))+(var2*(((((Pcx3*Pcz3)*m3)*costheta2)*costheta3)*sintheta2)));
	const double var23=((((((((((((((((Icxy3*sintheta2)-(Icyz2*costheta2))-(Icxz2*sintheta2))+((Icyz3*costheta2)*costheta3))+((Icxz3*costheta2)*sintheta3))-(var2*((Icxy3*sq(costheta3))*sintheta2)))-(((Pcy2*Pcz2)*m2)*costheta2))+(((Icxx3*costheta3)*sintheta2)*sintheta3))-(((Icyy3*costheta3)*sintheta2)*sintheta3))+(((Pcx3*Pcy3)*m3)*sintheta2))-(((Pcx2*Pcz2)*m2)*sintheta2))+((((Pcy3*Pcz3)*m3)*costheta2)*costheta3))-((((sq(Pcx3)*m3)*costheta3)*sintheta2)*sintheta3))+((((sq(Pcy3)*m3)*costheta3)*sintheta2)*sintheta3))+((((Pcx3*Pcz3)*m3)*costheta2)*sintheta3))-(var2*((((Pcx3*Pcy3)*m3)*sq(costheta3))*sintheta2)));
	const double var24=(((((Icyz3*sintheta2)*sintheta3)-((((Iczz3*costheta2)+((sq(Pcx3)*m3)*costheta2))+((sq(Pcy3)*m3)*costheta2))+((Icxz3*costheta3)*sintheta2)))-((((Pcx3*Pcz3)*m3)*costheta3)*sintheta2))+((((Pcy3*Pcz3)*m3)*sintheta2)*sintheta3));
	const double var25=((((var21*var11)+(var22*var14))+(var23*var16))+(var24*var18));
	const double var26=(var21-(var25*var11));
	const double var27=(var22-(var25*var14));
	const double var28=(var23-(var25*var16));
	const double var29=(var24-(var25*var18));
	const double var30=sqrt((((sq(var26)+sq(var27))+sq(var28))+sq(var29)));
	const double var31=(var26/var30);
	const double var32=(var13-(var19*var14));
	const double var33=(var27/var30);
	const double var34=(var15+(var19*var16));
	const double var35=(var28/var30);
 	const double var36=(var17-(var19*var18));
	const double var37=(var29/var30);
	const double var38=((((var20*var31)+(var32*var33))-(var34*var35))+(var36*var37));
	const double var39=(var20-(var38*var31));
	const double var40=(((((((((((((((((((((((((((((((((Icxx3*sintheta1)+(Iczz2*sintheta1))+(var7*((Pcy2*m2)*sintheta2)))-(var7*((Pcz3*m3)*sintheta2)))+((Icxy3*costheta1)*costheta2))-((Icxz2*costheta1)*costheta2))+((sq(Pcx2)*m2)*sintheta1))+((sq(Pcy2)*m2)*sintheta1))+((sq(Pcy3)*m3)*sintheta1))+((sq(Pcz3)*m3)*sintheta1))+((Icyz2*costheta1)*sintheta2))-((Icxx3*sq(costheta3))*sintheta1))+((Icyy3*sq(costheta3))*sintheta1))-(var7*((Pcx2*m2)*costheta2)))-(((Icyz3*costheta1)*costheta3)*sintheta2))-(var2*(((Icxy3*costheta3)*sintheta1)*sintheta3)))-(((Icxz3*costheta1)*sintheta2)*sintheta3))-(var2*(((Icxy3*costheta1)*costheta2)*sq(costheta3))))+(((sq(Pcx3)*m3)*sq(costheta3))*sintheta1))-(((sq(Pcy3)*m3)*sq(costheta3))*sintheta1))-(var7*(((Pcx3*m3)*costheta2)*costheta3)))+(var7*(((Pcy3*m3)*costheta2)*sintheta3)))+((((Icxx3*costheta1)*costheta2)*costheta3)*sintheta3))-((((Icyy3*costheta1)*costheta2)*costheta3)*sintheta3))+((((Pcx3*Pcy3)*m3)*costheta1)*costheta2))-((((Pcx2*Pcz2)*m2)*costheta1)*costheta2))+((((Pcy2*Pcz2)*m2)*costheta1)*sintheta2))-(var2*(((((Pcx3*Pcy3)*m3)*costheta3)*sintheta1)*sintheta3)))-(((((Pcx3*Pcz3)*m3)*costheta1)*sintheta2)*sintheta3))-(var2*(((((Pcx3*Pcy3)*m3)*costheta1)*costheta2)*sq(costheta3))))-(((((sq(Pcx3)*m3)*costheta1)*costheta2)*costheta3)*sintheta3))+(((((sq(Pcy3)*m3)*costheta1)*costheta2)*costheta3)*sintheta3))-(((((Pcy3*Pcz3)*m3)*costheta1)*costheta3)*sintheta2));
	const double var41=((((((((((((((((Icxy3*sintheta2)-(Icyz2*costheta2))-(Icxz2*sintheta2))+((Icyz3*costheta2)*costheta3))+((Icxz3*costheta2)*sintheta3))-(var2*((Icxy3*sq(costheta3))*sintheta2)))-(((Pcy2*Pcz2)*m2)*costheta2))+(((Icxx3*costheta3)*sintheta2)*sintheta3))-(((Icyy3*costheta3)*sintheta2)*sintheta3))+(((Pcx3*Pcy3)*m3)*sintheta2))-(((Pcx2*Pcz2)*m2)*sintheta2))+((((Pcy3*Pcz3)*m3)*costheta2)*costheta3))-((((sq(Pcx3)*m3)*costheta3)*sintheta2)*sintheta3))+((((sq(Pcy3)*m3)*costheta3)*sintheta2)*sintheta3))+((((Pcx3*Pcz3)*m3)*costheta2)*sintheta3))-(var2*((((Pcx3*Pcy3)*m3)*sq(costheta3))*sintheta2)));
	const double var42=(((((((((((Icxx3+Iczz2)+(sq(Pcx2)*m2))+(sq(Pcy2)*m2))+(sq(Pcy3)*m3))+(sq(Pcz3)*m3))-(Icxx3*sq(costheta3)))+(Icyy3*sq(costheta3)))-(Icxy3*sin((var2*q3))))+((sq(Pcx3)*m3)*sq(costheta3)))-((sq(Pcy3)*m3)*sq(costheta3)))-(((Pcx3*Pcy3)*m3)*sin((var2*q3))));
	const double var43=((((Icyz3*costheta3)+(Icxz3*sintheta3))+(((Pcy3*Pcz3)*m3)*costheta3))+(((Pcx3*Pcz3)*m3)*sintheta3));
	const double var44=((((var40*var11)+(var41*var14))+(var42*var16))-(var43*var18));
	const double var45=(var40-(var44*var11));
	const double var46=(var41-(var44*var14));
	const double var47=(var42-(var44*var16));
	const double var48=(var43+(var44*var18));
	const double var49=((((var45*var31)+(var46*var33))+(var47*var35))-(var48*var37));
	const double var50=(var45-(var49*var31));
	const double var51=(var46-(var49*var33));
	const double var52=(var47-(var49*var35));
	const double var53=(var48+(var49*var37));
	const double var54=sqrt((((sq(var50)+sq(var51))+sq(var52))+sq(var53)));
	const double var55=(var50/var54);
	const double var56=(var32-(var38*var33));
	const double var57=(var51/var54);
	const double var58=(var34+(var38*var35));
	const double var59=(var52/var54);
	const double var60=(var36-(var38*var37));
	const double var61=(var53/var54);
	const double var62=((((var39*var55)+(var56*var57))-(var58*var59))-(var60*var61));
	const double var63=(var39-(var62*var55));
	const double var64=(var56-(var62*var57));
	const double var65=(var58+(var62*var59));
	const double var66=(var60+(var62*var61));
	const double var67=sqrt((((sq(var63)+sq(var64))+sq(var65))+sq(var66)));
	const double var68=((var63/var67)/var67);
	const double var69=((var55-(var62*var68))/var54);
	const double var70=(((var31-(var38*var68))-(var49*var69))/var30);
	const double var71=((var64/var67)/var67);
	const double var72=((var57-(var62*var71))/var54);
	const double var73=(((var33-(var38*var71))-(var49*var72))/var30);
	const double var74=((var65/var67)/var67);
	const double var75=((var59+(var62*var74))/var54);
	const double var76=(((var35+(var38*var74))-(var49*var75))/var30);
	const double var77=((var66/var67)/var67);
	const double var78=((var61+(var62*var77))/var54);
	const double var79=(((var37-(var38*var77))+(var49*var78))/var30);


	nathanisdumb(0,0) = ((((var11-(var19*var68))-(var44*var69))-(var25*var70))/var10);
	nathanisdumb(0,1) = ((((var14-(var19*var71))-(var44*var72))-(var25*var73))/var10);
	nathanisdumb(0,2) = ((((var16+(var19*var74))-(var44*var75))-(var25*var76))/var10);
	nathanisdumb(0,3) = ((((var18-(var19*var77))+(var44*var78))-(var25*var79))/var10);
	nathanisdumb(1,0) = var70;
	nathanisdumb(1,1) = var73;
	nathanisdumb(1,2) = var76;
	nathanisdumb(1,3) = var79;
	nathanisdumb(2,0) = var69;
	nathanisdumb(2,1) = var72;
	nathanisdumb(2,2) = var75;
	nathanisdumb(2,3) = (-var78);
	nathanisdumb(3,0) = var68;
	nathanisdumb(3,1) = var71;
	nathanisdumb(3,2) = (-var74);
	nathanisdumb(3,3) = var77;

	return nathanisdumb;
}