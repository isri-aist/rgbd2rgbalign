
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "gflags/gflags.h"

#include <opencv2/opencv.hpp>

#include <ceres/cubic_interpolation.h>
#include "residue_pc2equi.h"

DEFINE_string(desired, "", "File of the reference image");
DEFINE_string(input, "", "File of the rgbd point cloud to align");
DEFINE_string(output, "", "File to which the aligned image should be written");
DEFINE_string(overlayedOutput, "", "File to which the aligned image is overlayed on the desired should be written");
DEFINE_double(sigma, -1.0, "Gaussian sigma to filter the images prior direct alignment");

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace cv;

#define INTRFACT 0.25

#define PI 3.1416

// Creates the alignment problem.
void CreateProblem(unsigned int nbPts, 
                   double *X, double *Y, double *Z,
                   double *I, 
                   const ceres::BiCubicInterpolator<ceres::Grid2D<double,1> >& equiImageInterpolator,
                   Problem* problem,
                   double *p, //relative camera pose
                   double *c,
                   double au_equi, double av_equi, double u0_equi, double v0_equi) 
{
  for (unsigned int iPts = 0 ; iPts < nbPts ; iPts++)
  {
    ceres::CostFunction* cost_function = residue::Create(
        X[iPts], Y[iPts], Z[iPts], I[iPts], equiImageInterpolator, au_equi, av_equi, u0_equi, v0_equi);

    problem->AddResidualBlock(
        cost_function, 
        NULL /* squared loss */, //new ceres::CauchyLoss(0.5), //
        p,
        c);//, aligned->MutablePixelFromLinearIndex(index));
  }

    problem->SetParameterization(p, new ceres::ProductParameterization(new ceres::IdentityParameterization(3), new ceres::QuaternionParameterization()));

   //problem->SetParameterBlockConstant(c);
   //problem->SetParameterBlockConstant(p);
}


void SolveProblem(Problem* problem, double *p, double *c) {

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true; 
  options.function_tolerance = 1e-6;//7; //12;
  options.max_num_iterations = 300;
  
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << "\n";

  std::cout << "p : " << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " " << p[4] << " " << p[5] << " " << p[6] << std::endl;
  std::cout << "c : " << c[0] << " " << c[1] << std::endl;
}


int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_desired.empty()) {
    std::cerr << "Please provide a reference image file name using -desired.\n";
    return 1;
  }

  if (FLAGS_input.empty()) {
    std::cerr << "Please provide a pts file name using -input.\n";
    return 1;
  }

  if (FLAGS_output.empty()) {
    std::cerr << "Please provide an output file name using -output.\n";
    return 1;
  }

  // Read the image
  Mat desired = imread(FLAGS_desired, IMREAD_GRAYSCALE), desiredOrig;
  
  desiredOrig = desired.clone();
  
  //ceres::examples::PGMImage<double> desired(FLAGS_desired);
  if (desired.cols == 0) {
    std::cerr << "Reading \"" << FLAGS_desired << "\" failed.\n";
    return 3;
  }

  // Read the point cloud
  std::ifstream ficPC(FLAGS_input);
  unsigned int nbPts;
  ficPC >> nbPts;

  std::cout << "nbPts: " << nbPts << std::endl;

  double *X = new double[nbPts], *Y = new double[nbPts], *Z = new double[nbPts], *I = new double[nbPts];
  unsigned R, G, B;
  int Di;
  for (unsigned int iPts = 0 ; iPts < nbPts ; iPts++)
  {
    ficPC >> X[iPts] >> Y[iPts] >> Z[iPts] >> Di >> R >> G >> B;
    I[iPts] = 0.299*R + 0.587*G + 0.114*B; 
  }

  std::cout << " last 3D point " << X[nbPts-1] << " " << Y[nbPts-1] << " " << Z[nbPts-1] << std::endl;
    
  namedWindow("Desired", WINDOW_AUTOSIZE );
  imshow("Desired", desired);
  waitKey(0);
  
  desired.convertTo(desired, CV_64FC1);
  
  if (FLAGS_sigma <= 0.0f) {
    std::cerr << "Working with original images.\n";
  }
  else
  {
  	GaussianBlur(desired, desired, Size(0,0), FLAGS_sigma, FLAGS_sigma, BORDER_REPLICATE);
  	std::cerr << "Working with Gaussian filtered images of standard deviation " << FLAGS_sigma <<"\n";
  }
  
  // Create the data term
  
  unsigned int totalElements = desired.total()*desired.channels(); // Note: image.total() == rows*cols.
  cv::Mat flat_desired = desired.reshape(1, totalElements); // 1xN mat of 1 channel, O(1) operation
  if(!desired.isContinuous()) {
        flat_desired = flat_desired.clone(); // O(N),
    }

  double p[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}; //frame change: current -> desired
  //after initial alignment on frame0000 with sigma=2
  //double p[7] = {1.51209e-05, -0.13206, 0.0927016, 0.999225, -0.0343738, 0.019157, -0.000357765};
  //optimum of frame0000
  //double p[7] = {0.00415287, -0.131965, 0.0988572, 0.999248, -0.0344592, 0.0177416, -0.000341746};
  //init for 1097
  //double p[7] = {0.1, 0.1, 0.1, 1.0, 0.0, 0.0, 0.0}; //frame change: current -> desired
  //optimum for frame1097 with 544
  //double p[7] = {0.0600481, 0.0130714, 0.11973, 0.99994, -0.00800297, 0.0069233, 0.00297426};
  //optimum for frame0000 when initialized from optimum for frame1097
  //double p[7] = {0.0722559, 0.00824957, 0.0862648, 0.999921, -0.0109627, 0.00261817, 0.00557741};

  double affine_photometric_model[2] = {1., 0};

  ceres::Grid2D<double, 1> array(flat_desired.ptr<double>(0), 0, desired.rows, 0, desired.cols);
  ceres::BiCubicInterpolator<ceres::Grid2D<double, 1> > equiImageInterpolator(array);

	double au_equi = desired.cols*0.5/PI;
	double av_equi = 0.5*desired.rows/(0.5*PI);
	double u0_equi = desired.cols * 0.5;
	double v0_equi = desired.rows * 0.5;

  ceres::Problem problem;
  CreateProblem(nbPts, X, Y, Z, I, equiImageInterpolator, &problem, p, affine_photometric_model, au_equi, av_equi, u0_equi, v0_equi);

  SolveProblem(&problem, p, affine_photometric_model);

  //compute the resulting aligned image by warping the desired image to the input
  Mat aligned(desired.rows, desired.cols, CV_64FC1, Scalar(0.0));
  Mat overlayed = desired.clone();

for (unsigned int iPts = 0 ; iPts < nbPts ; iPts++)
{
  double dX_[3] = {X[iPts], Y[iPts], Z[iPts]};
      double cX_[3];

      ceres::QuaternionRotatePoint(p+3, dX_, cX_);

      cX_[0] += p[0];
      cX_[1] += p[1];
      cX_[2] += p[2];

      //project in the equirectangular image

      double rho = sqrt(cX_[0]*cX_[0] + cX_[1]*cX_[1] + cX_[2]*cX_[2]);
      double Xs = cX_[0] / rho;
      double Ys = cX_[1] / rho;
      double Zs = cX_[2] / rho;
      
      //double theta = atan2(Ys,Xs); //Azimuth entre -pi et pi
    	//double phi = atan2(sqrt(Xs*Xs + Ys*Ys), Zs); //Elevation entre -pi/2 et pi/2
    	//double theta = atan2(Ys, Xs); //Azimuth entre -pi et pi
    	//double phi = asin(Zs);
      double theta = atan2(Xs, Zs); //Azimuth entre -pi et pi
    	double phi = atan2(Ys, sqrt(Xs*Xs + Zs*Zs));  

    	
    	//std::cout << phi << std::endl;
    	
    	double u = au_equi*theta + u0_equi;
    	double v = av_equi*phi + v0_equi;

      if( (u >= 0) && (u < aligned.cols) && (v >= 0) && (v < aligned.rows) )
      {
        aligned.at<double>(v, u) = (I[iPts]-affine_photometric_model[1])/affine_photometric_model[0];
        if (!FLAGS_overlayedOutput.empty()) 
          overlayed.at<double>(v, u) = (overlayed.at<double>(v, u)+aligned.at<double>(v, u))*0.5;
      }
}

double equiRpc[3][3];
ceres::QuaternionToRotation<double>(p+3, equiRpc[0]);

std::cout << "equiMpc = " << std::endl;
std::cout << equiRpc[0][0] << "\t\t" << equiRpc[0][1] << "\t\t" << equiRpc[0][2] << "\t\t" << p[0] << std::endl;
std::cout << equiRpc[1][0] << "\t\t" << equiRpc[1][1] << "\t\t" << equiRpc[1][2] << "\t\t" << p[1] << std::endl;
std::cout << equiRpc[2][0] << "\t\t" << equiRpc[2][1] << "\t\t" << equiRpc[2][2] << "\t\t" << p[2] << std::endl;
std::cout << "0\t\t0\t\t0\t\t1" << std::endl;

aligned.convertTo(aligned, CV_8UC1);
if (!FLAGS_output.empty()) {
  CHECK(imwrite(FLAGS_output, aligned))
      << "Writing \"" << FLAGS_output << "\" failed.";
}

if (!FLAGS_overlayedOutput.empty()) 
{
    overlayed.convertTo(overlayed, CV_8UC1);
      CHECK(imwrite(FLAGS_overlayedOutput, overlayed))
      << "Writing \"" << FLAGS_overlayedOutput << "\" failed.";
}
	/*
  //compute the resulting aligned image
  ceres::Grid2D<double, 1> arrayDes(desired.ptr<double>(0), 0, desired.rows, 0, desired.cols);
  ceres::BiCubicInterpolator<ceres::Grid2D<double, 1> > desiredInterpolator(arrayDes);

  int index;
  double ud, vd;
  double I_aligned;
  Eigen::Matrix<double, 3, 3> iH = H.inverse();
  double *ih = iH.data();
  for (unsigned int v = 0 ; v < aligned.rows ; v++)
    for (unsigned int u = 0 ; u < aligned.cols ; u++)
    {
      ud = (u*ih[0] + v*ih[3] + ih[6]) / (u*ih[2] + v*ih[5] + 1);
      vd = (u*ih[1] + v*ih[4] + ih[7]) / (u*ih[2] + v*ih[5] + 1);

      desiredInterpolator.Evaluate(vd, ud, &I_aligned );

      aligned.at<double>(v, u) = I_aligned;
      desired.PixelFromLinearIndex(desired.LinearIndex(ud,vd));//
    }

	aligned.convertTo(aligned, CV_8UC1);
  if (!FLAGS_output.empty()) {
    CHECK(imwrite(FLAGS_output, aligned))
        << "Writing \"" << FLAGS_output << "\" failed.";
  }
  */

  return 0;
}


