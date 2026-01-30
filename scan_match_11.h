#ifndef ELEVEN_H // if <anyname> variable isn't defined...
#define ELEVEN_H // ...define it now. Can't enter this if block twice!
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include "pose.h"
using namespace std;
using namespace Eigen;

// Use pose.h pose.cpp instead
/* struct Pose2D { */
/*     double x, y, theta; */
/*     Pose2D(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {} */
/* }; */

struct Point2D {
    double x, y;
    Point2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
};

struct GaussianCell {
    Vector2d mean;
    Matrix2d covariance;
    Matrix2d covInv;
    double det;
    GaussianCell(const Vector2d& m, const Matrix2d& cov) : mean(m), covariance(cov), covInv((Eigen::Matrix2d() << cov(1,1), -cov(0,1), -cov(0,1),  cov(0,0)).finished()*1/cov.determinant()), det(cov.determinant()) {}

};

struct PairHash {
    std::size_t operator()(const std::pair<int,int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

// Type definitions
typedef MatrixXd Scan;  // Nx2 matrix where each row is (x,y)
typedef std::unordered_map<std::pair<int,int>, GaussianCell, PairHash> NDTGrid;


class NDTScanMatcher {
private:
    // Constants defined as static const instead of constexpr
    static const double ID2_REG;
    static const double D1;
    static const double D2;
    static const double D3;
    static const double MAX_GRADIENT_NORM;
    static const double CONVERGENCE_TOL_LAMBDA;
    static const double CONVERGENCE_TOL_GRAD;
    static const double CONVERGENCE_TOL_SCORE;
    
public:

	// NelderMead
	std::vector<std::vector<double>> neldermead(
		const Scan&, 
		const Scan&, 
		double,
		const NDTGrid&,
		double, 
		double,
		double
	);

    NDTGrid computeNDTGrid(const Scan&, double);
    
    Scan transformScan(const Scan&, double, double, double);    
    Scan transformScanToPose(const Scan&, const Pose2D&);     
    double computeNDTScore(const Scan&, const NDTGrid&, double);    
private:
    double getNewtonData(Matrix3d&, Vector3d&, double,double, double,
		    Matrix3d&, const Vector2d&,
		    const GaussianCell&);    
    void makePositiveDefinite(Matrix3d&, bool&, int );    
public:
    void ndtScanMatchHP(const Scan&, const Scan&, 
		    double, Pose2D&, Matrix3d&,
		    int , double , 
		    double , double , 
		    double , bool );
    void transformScanInPlace(Scan&, const Scan&,
		    double, double, double);

    // Golden section search for optimal angle (faster than Nelder-Mead for 1D)
    double goldenSectionAngleSearch(
        const Scan& scan1,
        const NDTGrid& ndt_grid,
        double grid_size,
        double tx,
        double ty,
        double angle_min,
        double angle_max,
        double tol = 0.005
    );

};

#endif
