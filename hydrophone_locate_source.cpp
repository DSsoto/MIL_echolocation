#include "ceres/ceres.h"
#include "glog/logging.h"
#include <string>
#include <cstdlib>
#include <cmath>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

#define speed_of_sound 1482.00 // m/s

struct Hydrophone1Cost {
public:
	Hydrophone1Cost(double ts1){ tstamp = ts1;}
	double tstamp;
	double hp_pose[3] = {1.0, 0.0, 0.0};  // [x, y, z]
	/* TODO:
		Set all the actual positions of the hydrophones in their respective hp_pose 
	*/
	
	template <typename T> bool operator() (const T* const x, const T* const y, 
							 const T* const z, const T* const t, T* residual) const {
		T est_dist = sqrt(pow(x[0] - hp_pose[0], 2.0) + pow(y[0] - hp_pose[1], 2.0) + pow(z[0] - hp_pose[2], 2.0));
		T time_of_flight = est_dist / speed_of_sound;
		residual[0] = tstamp - t[0] - time_of_flight;
		return true;
	}
}; 
struct Hydrophone2Cost {
public:
	Hydrophone2Cost(double ts2){ tstamp = ts2;}
	double tstamp;
	double hp_pose[3] = {0.0, 1.0, 0.0};
	template <typename T> bool operator() (const T* const x, const T* const y, 
							 const T* const z, const T* const t, T* residual) const {
		T est_dist = sqrt(pow(x[0] - hp_pose[0], 2.0) + pow(y[0] - hp_pose[1], 2.0) + pow(z[0] - hp_pose[2], 2.0));
		T time_of_flight = est_dist / speed_of_sound;
		residual[0] = tstamp - t[0] - time_of_flight;
		return true;
	}
}; 
struct Hydrophone3Cost {
public:
	Hydrophone3Cost(double ts3){ tstamp = ts3;}
	double tstamp;
	double hp_pose[3] = {0.0, 0.0, 1.0};
	template <typename T> bool operator() (const T* const x, const T* const y, 
							 const T* const z, const T* const t, T* residual) const {
		T est_dist = sqrt(pow(x[0] - hp_pose[0], 2.0) + pow(y[0] - hp_pose[1], 2.0) + pow(z[0] - hp_pose[2], 2.0));
		T time_of_flight = est_dist / speed_of_sound;
		residual[0] = tstamp - t[0] - time_of_flight;
		return true;
	}
}; 
struct Hydrophone4Cost {
public:
	Hydrophone4Cost(double ts4){ tstamp = ts4;}
	double tstamp;
	double hp_pose[3] = {1.0, 1.0, 1.0};
	template <typename T> bool operator() (const T* const x, const T* const y, 
							 const T* const z, const T* const t, T* residual) const {
		T est_dist = sqrt(pow(x[0] - hp_pose[0], 2.0) + pow(y[0] - hp_pose[1], 2.0) + pow(z[0] - hp_pose[2], 2.0));
		T time_of_flight = est_dist / speed_of_sound;
		residual[0] = tstamp - t[0] - time_of_flight;
		return true;
	}
}; 

int main(int argc, char** argv){

	std::string help = 
		"Locates the 3D position of a sound source\n"
		"Arguments: <timestamp1> <timestamp2> <timestamp3> <timestamp4>\n"
		"Note: \n\ttimestamps must be in the correct order to obtain meaningful result\n";
	
	if(std::strcmp(argv[1], "-h") == 0){
		std::cout << help << std::endl;
	} else if(argc < 5){
		std::cout << "Usage:\n" << argv[0] 
			<< " <timestamp1> <timestamp2> <timestamp3> <timestamp4>"
			<< std::endl;
	}

	double ts1 = std::atof(argv[1]);
	double ts2 = std::atof(argv[2]);
	double ts3 = std::atof(argv[3]);
	double ts4 = std::atof(argv[4]);

	const double initial_x = 10;
	const double initial_y = 0;
	const double initial_z = 0;
	const double initial_t = ts1;
	double x = initial_x;
	double y = initial_y;
	double z = initial_z;
	double t = initial_t;

	Problem problem;
	CostFunction* h1cost = new AutoDiffCostFunction<Hydrophone1Cost ,1 ,1, 1, 1, 1>(new Hydrophone1Cost(ts1));
	CostFunction* h2cost = new AutoDiffCostFunction<Hydrophone2Cost ,1 ,1, 1, 1, 1>(new Hydrophone2Cost(ts2));
	CostFunction* h3cost = new AutoDiffCostFunction<Hydrophone3Cost ,1 ,1, 1, 1, 1>(new Hydrophone3Cost(ts3));
	CostFunction* h4cost = new AutoDiffCostFunction<Hydrophone4Cost ,1 ,1, 1, 1, 1>(new Hydrophone4Cost(ts4));
	problem.AddResidualBlock(h1cost, NULL, &x, &y, &z, &t);
	problem.AddResidualBlock(h2cost, NULL, &x, &y, &z, &t);
	problem.AddResidualBlock(h3cost, NULL, &x, &y, &z, &t);
	problem.AddResidualBlock(h4cost, NULL, &x, &y, &z, &t);

	Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	std::cout << "Initial x = " << x
	          << ", y = " << y
	          << ", z = " << z
	          << ", t = " << t
	          << "\n";
	// Run the solver!
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
	std::cout << "Final x = " << x
	          << ", y = " << y
	          << ", z = " << z
	          << ", t = " << t
	          << "\n";
	return 0;
}