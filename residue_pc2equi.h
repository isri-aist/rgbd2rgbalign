#ifndef residue_pc2equi_H_
#define residue_pc2equi_H_

#include <ceres/cubic_interpolation.h>
#include <ceres/rotation.h>

struct residue
{
    /// Set the constants of the problem
    residue(const double & X, const double & Y, const double & Z, const double & I,
               const ceres::BiCubicInterpolator<ceres::Grid2D<double,1> > & equiImage, const double & au, const double & av, const double & u0, const double & v0) : X(X), Y(Y), Z(Z), I_observed(I), equiImage(equiImage), au(au), av(av), u0(u0), v0(v0)
    {
    }

    template <typename T>
    bool operator()(const T* const p, const T* const c, T* residual) const
    {
      T I_predicted;

      //transform the 3D point according to the current rigid transformation
      T dX_[3] = {T(X), T(Y), T(Z)};
      T cX_[3];

      ceres::QuaternionRotatePoint(p+3, dX_, cX_);

      cX_[0] += p[0];
      cX_[1] += p[1];
      cX_[2] += p[2];

      //project in the equirectangular image

      T rho = sqrt(cX_[0]*cX_[0] + cX_[1]*cX_[1] + cX_[2]*cX_[2]);
      T Xs = cX_[0] / rho;
      T Ys = cX_[1] / rho;
      T Zs = cX_[2] / rho;
      
      //Same frame as in ICIP 2022
    	//T theta = atan2(Ys, Xs); //Azimuth entre -pi et pi
    	//T phi = asin(Zs);
      //Same frame as in libPeR
    	T theta = atan2(Xs, Zs); //Azimuth entre -pi et pi
    	T phi = atan2(Ys, sqrt(Xs*Xs + Zs*Zs));  

    	//std::cout << phi << std::endl;
    	
    	T u = au*theta + u0;
    	T v = av*phi + v0;
    	
    	//std::cout << v << std::endl;
      
      equiImage.Evaluate(v, u, &I_predicted ); // row, col, interpolated
      
      residual[0] = c[0]*(I_predicted)+c[1] - T(I_observed);

      return true;
    }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double & X, const double & Y, const double & Z, const double & I,
               const ceres::BiCubicInterpolator<ceres::Grid2D<double,1> > & imageEqui, const double & au, const double & av, const double & u0, const double & v0) {
     return (new ceres::AutoDiffCostFunction<residue, 1, 7, 2>(
                 new residue(X, Y, Z, I, imageEqui, au, av, u0, v0)));
   }

    double I_observed;

    double X, Y, Z;
    double au, av;
    double u0, v0;

    const ceres::BiCubicInterpolator< ceres::Grid2D<double,1> > & equiImage;
};

#endif  // residue_pc2equi_H_
