#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <Eigen/Dense>
#include "scan_match_11.h"

using namespace std;
using namespace Eigen;

// Scan match in place to avoid reallocations
void NDTScanMatcher::transformScanInPlace(Scan& output, const Scan& scan, 
                                          double tx, double ty, double phi) {
    double c = cos(phi);
    double s = sin(phi);
    
    for (int i = 0; i < scan.rows(); ++i) {
        double x = scan(i, 0);
        double y = scan(i, 1);
        output(i, 0) = c*x - s*y + tx;
        output(i, 1) = s*x + c*y + ty;
    }
}


// NelderMead
std::vector<std::vector<double>> NDTScanMatcher::neldermead(
	const Scan& scan2, 
	const Scan& scan1, 
	double grid_size,
	const NDTGrid& ndt_grid,
	double initial_dx = 0.0, 
	double initial_dy = 0.0, 
	double initial_angle = 0.0
) {
	// Nelder-Mead typical parameters from Wikipedia
	const double alpha = 1.0;
	const double gamma = 2.0;
	const double rho = 0.5;
	const double sigma = 0.5;
	
	double x1 = 0.0 + initial_angle;
	double x2 = 0.05 + initial_angle;
	
	std::vector<std::vector<double>> iterates;
	double result=0.0;
	
	// Define objective function (C++11 compatible)
	Scan transformed_scan(scan1.rows(), 2);

	std::function<double(double)> fn = [&](double x) -> double {
		transformScanInPlace(transformed_scan,scan1, initial_dx, initial_dy, x);
		return computeNDTScore(transformed_scan, ndt_grid, grid_size);
	};
	
	double fc = 0.0;
	double fr = 0.0;
	
	while (std::abs(x1 - x2) > 0.005) {
		//iterates.push_back({x1, x2});
		result = (x1+x2)*0.5;

		
		double f1 = fn(x1);
		double f2 = fn(x2);
		
		// Ensure x1 is the better point (lower function value)
		if (f1 > f2) {
			std::swap(x1, x2);
			std::swap(f1, f2);
		}
		
		// Reflection
		double xr = x1 + alpha * (x1 - x2);
		fr = fn(xr);
		
		if (fr < f1) {
			// Expansion
			double xe = x1 + gamma * (x2 - x1);
			double fe = fn(xe);
			
			if (fe < fr) {
				x2 = x1;
				x1 = xe;
				continue;
			} else {
				x2 = xr;
				continue;
			}
		} else {  // fr >= f1
			if (fr > f2) {
				// Outside contraction
				double xc = x1 + rho * (xr - x1);
				fc = fn(xc);
				
				if (fc < fr) {
					x2 = xc;
					continue;
				} else {
					// Shrink
					x2 = x1 + sigma * (x2 - x1);
					continue;
				}
			} else {
				// Inside contraction
				double xc = x1 + rho * (x2 - x1);
				fc = fn(xc);
				
				if (fc < fr) {
					x2 = xc;
					continue;
				} else {
					// Shrink
					x2 = x1 + sigma * (x2 - x1);
					continue;
				}
			}
		}
	}
	
	iterates.push_back({result});
	return iterates;
}




NDTGrid NDTScanMatcher::computeNDTGrid(const Scan& scan, double gridSize) {
	NDTGrid ndtGrid;
	
	// Compute grid indices for each point
	MatrixXi gridIndices = (scan / gridSize).array().floor().cast<int>();
	
	// Group points by grid cell
	unordered_map<pair<int,int>, vector<int>, PairHash> cellPoints;
	
	for (int i = 0; i < gridIndices.rows(); ++i) {
		pair<int,int> cellKey = make_pair(gridIndices(i,0), gridIndices(i,1));
		cellPoints[cellKey].push_back(i);
	}
	
	// Compute Gaussian distributions for each cell
	for (unordered_map<pair<int,int>, vector<int>, PairHash>::const_iterator it = cellPoints.begin(); 
		 it != cellPoints.end(); ++it) {
		const pair<int,int>& cellKey = it->first;
		const vector<int>& pointIndices = it->second;
		
		if (pointIndices.size() > 2) {  // At least 3 points to compute covariance
			MatrixXd cellPointsMat(pointIndices.size(), 2);
			for (size_t i = 0; i < pointIndices.size(); ++i) {
				cellPointsMat.row(i) = scan.row(pointIndices[i]);
			}
			
			Vector2d mean = cellPointsMat.colwise().mean();
			MatrixXd centered = cellPointsMat.rowwise() - mean.transpose();
			Matrix2d covariance = (centered.transpose() * centered) / (pointIndices.size() - 1);
			
			// Regularize covariance to avoid singular matrices
			covariance += Matrix2d::Identity() * ID2_REG;
			
			ndtGrid.insert(make_pair(cellKey, GaussianCell(mean, covariance)));
		}
	}
	
	return ndtGrid;
}

Scan NDTScanMatcher::transformScan(const Scan& scan, double tx, double ty, double phi) {
	Matrix2d rotation;
	rotation << cos(phi), -sin(phi),
			   sin(phi),  cos(phi);
	
	Vector2d translation(tx, ty);
	
	Scan transformed = (rotation * scan.transpose()).transpose();
	transformed.rowwise() += translation.transpose();
	
	return transformed;
}

Scan NDTScanMatcher::transformScanToPose(const Scan& scan, const Pose2D& pose) {
	return transformScan(scan, pose.x_, pose.y_, pose.theta_);
}

double NDTScanMatcher::computeNDTScore(const Scan& scan, const NDTGrid& ndtGrid, double gridSize) {
	MatrixXi gridIndices = (scan / gridSize).array().floor().cast<int>();
	
	int nPoints = scan.rows();
	//VectorXd scores = VectorXd::Constant(nPoints, D3);  // Default score for missing cells
	double totalScore = scan.rows() * D3;
	
	// Find valid cells
	for (int i = 0; i < nPoints; ++i) {
		pair<int,int> cellKey = make_pair(gridIndices(i,0), gridIndices(i,1));
		NDTGrid::const_iterator it = ndtGrid.find(cellKey);
		
		if (it != ndtGrid.end()) {
			const GaussianCell& cell = it->second;
			Vector2d point = scan.row(i);
			Vector2d v = point - cell.mean;
			
			// Compute Mahalanobis distance efficiently
			double d = (v(0)*v(0)*cell.covariance(1,1) + 
					   v(1)*v(1)*cell.covariance(0,0) - 
					   2*cell.covariance(0,1)*v(0)*v(1)) / cell.det;
			
			totalScore -= D1 * exp(-0.5 * D2 * d);
		}
	}
	
	return totalScore;
}

double NDTScanMatcher::getNewtonData(Matrix3d& Amat, 
					Vector3d& bvec, 
					double phi,
					double cphi,
					double sphi,
					Matrix3d& tempSquare, 
					const Vector2d& point,
					const GaussianCell& cell) {
	
	
	Vector2d v = point - cell.mean;
	double d = (v(0)*v(0)*cell.covariance(1,1) + 
			   v(1)*v(1)*cell.covariance(0,0) - 
			   2*cell.covariance(0,1)*v(0)*v(1)) / cell.det;
	
	double e = -D1 * D2 / 2 * exp(-0.5 * d * D2);
	double score = D3 - D1 * exp(-0.5 * D2 * d);
	
	// Matrix<double, 2, 3> Jp;
	// Jp << 1, 0, -point(0)*sphi - point(1)*cphi,
	//    0, 1,  point(0)*cphi - point(1)*sphi;
	// Third row/column requires actual computation with Jp(0,2) and Jp(1,2)
	double jp02 = -point(0)*sphi - point(1)*cphi;
	double jp12 = point(0)*cphi - point(1)*sphi;

	Vector3d tempVec;
	double covInvV0 = cell.covInv(0,0)*v(0)+cell.covInv(0,1)*v(1);
	double covInvV1 = cell.covInv(1,0)*v(0)+cell.covInv(1,1)*v(1);
	tempVec(0) = covInvV0;
	tempVec(1) = covInvV1;
	tempVec(2) = jp02*covInvV0+ jp12*covInvV1;

	bvec += -e * tempVec;
	// Top-left 2x2 block is just covInv (since first two cols of Jp are identity)
	Amat(0,0) += -e * cell.covInv(0,0);
	Amat(0,1) += -e * cell.covInv(0,1);
	Amat(1,0) += -e * cell.covInv(1,0);
	Amat(1,1) += -e * cell.covInv(1,1);


	Amat(0,2) += -e * (cell.covInv(0,0)*jp02 + cell.covInv(0,1)*jp12);
	Amat(1,2) += -e * (cell.covInv(1,0)*jp02 + cell.covInv(1,1)*jp12);
	Amat(2,2) += -e * (jp02*cell.covInv(0,0)*jp02 +
			jp02*cell.covInv(0,1)*jp12 +
			jp12*cell.covInv(1,0)*jp02 +
			jp12*cell.covInv(1,1)*jp12);

	// Make symmetric
	Amat(2,0) = Amat(0,2);
	Amat(2,1) = Amat(1,2);
	double scale = e * D2 * 0.5;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Amat(i,j) += scale * tempVec(i) * tempVec(j);
		}
	}
	//Amat += -e * Jp.transpose() * cell.covInv * Jp;
	Amat(2,2) += -e * (-v(0)*jp12 + v(1)*jp02);
	//
	//tempSquare = tempVec * tempVec.transpose();
	//Amat += e * D2 / 2 * tempSquare;

	return score;
}

void NDTScanMatcher::makePositiveDefinite(Matrix3d& A, bool& isPD, int maxAttempts = 100) {
	double regParam = 1e-6;
	isPD = false;
	
	for (int attempt = 0; attempt < maxAttempts; ++attempt) {
		LLT<Matrix3d> llt(A);
		if (llt.info() == Success) {
			isPD = true;
			return;
		}
		A += regParam * Matrix3d::Identity();
		regParam *= 10;
	}
}

void NDTScanMatcher::ndtScanMatchHP(const Scan& scan2, const Scan& scan1, 
				   double gridSize, Pose2D& result, Matrix3d& hessian,
				   int maxIters = 60, double tol = 1e-6, 
				   double txInit = 0.0, double tyInit = 0.0, 
				   double phiInit = 0.0, bool debug = false) {
	// std::cout << "Initial angle: "  << phiInit << "\n";
	
	// Compute NDT grid for the reference scan
	NDTGrid ndtGrid = computeNDTGrid(scan2, gridSize);
	
	// Initialize parameters
	double tx = txInit, ty = tyInit, phi = phiInit;
	
	
	Matrix3d tempSquare = Matrix3d::Zero();
	double prevScore2 = 0.0;
	Matrix3d Amat = Matrix3d::Zero();
	Vector3d bvec = Vector3d::Zero();
	
	for (int count = 0; count < maxIters; ++count) {
		Scan transformedScan = transformScan(scan1, tx, ty, phi);
		double cphi = cos(phi);
		double sphi = sin(phi);
		Amat.setZero();
		bvec.setZero();
		
		double score2 = 0.0;
		
		MatrixXi gridIndices = (transformedScan / gridSize).array().floor().cast<int>();
		
		for (int i = 0; i < transformedScan.rows(); ++i) {
			Vector2d point = transformedScan.row(i);
			pair<int,int> cellKey = make_pair(gridIndices(i,0), gridIndices(i,1));
			
			NDTGrid::const_iterator it = ndtGrid.find(cellKey);
			if (it != ndtGrid.end()) {
				const GaussianCell& cell = it->second;
				score2 += getNewtonData(Amat, bvec, phi,cphi,sphi, tempSquare, 
									  point, cell);
			}
		}
		
		// Make Hessian positive definite
		Matrix3d AmatPD = Amat;
		bool isPD;
		makePositiveDefinite(AmatPD, isPD);

		Vector3d sol;
		if (isPD) {
			try {
				sol = AmatPD.llt().solve(bvec);
			} catch (...) {
				sol = bvec;
			}
		} else {
			sol = bvec;
		}

		// Gradient clipping
		double normSol = sol.norm();
		if (normSol > MAX_GRADIENT_NORM) {
			sol = sol / normSol * MAX_GRADIENT_NORM;
		}

		double gradTx = sol(0);
		double gradTy = sol(1);
		double gradPhi = sol(2);

		// Backtracking line search
		double alpha = 1.0;
		double c = 0.5;
		double b = 0.01;

		double score3 = computeNDTScore(transformedScan, ndtGrid, gridSize);
		Scan transformedScanTest = transformScan(scan1, tx - alpha*gradTx, 
											   ty - alpha*gradTy, phi - alpha*gradPhi);
		double scoreNext = computeNDTScore(transformedScanTest, ndtGrid, gridSize);
		
		double lambda2 = bvec.transpose() * sol;
		bool flag = scoreNext > score3 - b*alpha*lambda2;
		
		while (flag && alpha > 1e-8) {
			alpha *= c;
			transformedScanTest = transformScan(scan1, tx - alpha*gradTx, ty - alpha*gradTy, phi - alpha*gradPhi);
			scoreNext = computeNDTScore(transformedScanTest, ndtGrid, gridSize);
			flag = scoreNext > score3 - b*alpha*lambda2;
		}

		
		// Update parameters
		tx -= gradTx * alpha;
		ty -= gradTy * alpha;
		phi -= gradPhi * alpha;
		
		// Check for convergence
		if (abs(lambda2) < CONVERGENCE_TOL_LAMBDA) {
			//if (debug) cout << "Convergence: lambda^2 = " << lambda2 << endl;
			break;
		}
		if (bvec.norm() < tol) {
			//if (debug) cout << "Convergence: gradient norm = " << bvec.norm() << endl;
			break;
		}
		if (count > 1 && abs(score2 - prevScore2) < CONVERGENCE_TOL_SCORE) {
			//if (debug) cout << "Convergence: score change = " << abs(score2 - prevScore2) << endl;
			break;
		}
		prevScore2 = score2;
	}
	
	result = Pose2D(tx, ty, phi);
	hessian = Amat;
}

// Define static constants outside class
const double NDTScanMatcher::ID2_REG = 1e-6;
const double NDTScanMatcher::D1 = 1.4663370687934272;
const double NDTScanMatcher::D2 = 0.5643202489410892;
const double NDTScanMatcher::D3 = 1.2039728043259361;
const double NDTScanMatcher::MAX_GRADIENT_NORM = 0.2;
const double NDTScanMatcher::CONVERGENCE_TOL_LAMBDA = 0.01;
const double NDTScanMatcher::CONVERGENCE_TOL_GRAD = 1e-6;
const double NDTScanMatcher::CONVERGENCE_TOL_SCORE = 1e-2;

