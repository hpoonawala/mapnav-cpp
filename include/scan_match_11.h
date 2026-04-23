#ifndef MAPNAV_SCAN_MATCH_11_H // if <anyname> variable isn't defined...
#define MAPNAV_SCAN_MATCH_11_H // ...define it now. Can't enter this if block twice!
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <Eigen/Dense>
#include "../include/pose.h"

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
	Eigen::Vector2d mean;
    Eigen::Matrix2d covariance;
    Eigen::Matrix2d covInv;
    double det;
    double invdet;
    GaussianCell() : mean(Eigen::Vector2d::Zero()), covariance(Eigen::Matrix2d::Zero()), covInv(Eigen::Matrix2d::Zero()), det(1.0), invdet(1.0) {}
    GaussianCell(const Eigen::Vector2d& m, const Eigen::Matrix2d& cov) : mean(m), covariance(cov), covInv((Eigen::Matrix2d() << cov(1,1), -cov(0,1), -cov(0,1),  cov(0,0)).finished()*1/cov.determinant()), det(cov.determinant()), invdet(1/cov.determinant()) {}

};

// Type definitions
typedef Eigen::MatrixXd Scan;  // Nx2 matrix where each row is (x,y)

struct NDTGrid {
    std::vector<GaussianCell> cells;
    std::vector<char> valid;
    int min_i, min_j;
    int width, height;

    NDTGrid() : min_i(0), min_j(0), width(0), height(0) {}

    const GaussianCell* find(int i, int j) const {
        int ci = i - min_i;
        int cj = j - min_j;
        if (ci < 0 || ci >= width || cj < 0 || cj >= height) return NULL;
        int idx = ci * height + cj;
        if (!valid[idx]) return NULL;
        return &cells[idx];
    }
};


class NDTScanMatcher {
private:
    // Constants defined as static const instead of constexpr
    static const double ID2_REG;
    static const double D1;
    static const double D2;
    static const double D3;
    static const double NEG_HALF_D2;
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
    
    double computeNDTScore(const Scan&, const NDTGrid&, double invGridSize, Eigen::MatrixXi& gridIndices);    
private:
    double getNewtonData(Eigen::Matrix3d&, Eigen::Vector3d&, double,double, double,
		    Eigen::Matrix3d&, const Eigen::Vector2d&,
		    const GaussianCell&);    
    void makePositiveDefinite(Eigen::Matrix3d&, bool&, int );    
public:
    void ndtScanMatchHP(const Scan&, const Scan&, 
		    double, Pose2D&, double&, Eigen::Matrix3d&,
		    int , double , 
		    double , double , 
		    double , bool );

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
