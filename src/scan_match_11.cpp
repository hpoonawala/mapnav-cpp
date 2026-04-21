#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <Eigen/Dense>
#include "../include/scan_match_11.h"
#include "../include/timer.h"

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
	MatrixXi gridIndicesBuf(scan1.rows(), 2);
	double invGridSize = 1.0 / grid_size;

	std::function<double(double)> fn = [&](double x) -> double {
		transformScanInPlace(transformed_scan,scan1, initial_dx, initial_dy, x);
		return computeNDTScore(transformed_scan, ndt_grid, invGridSize, gridIndicesBuf);
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


// Golden section search for optimal angle
// More efficient than Nelder-Mead for 1D optimization: only 1 function eval per iteration
double NDTScanMatcher::goldenSectionAngleSearch(
	const Scan& scan1,
	const NDTGrid& ndt_grid,
	double grid_size,
	double tx,
	double ty,
	double angle_min,
	double angle_max,
	double tol
) {
	// Golden ratio constants
	const double invphi = 0.6180339887498949;   // 1/phi = (sqrt(5)-1)/2
	const double invphi2 = 0.3819660112501051;  // 1/phi^2

	// Pre-allocate transformed scan buffer
	Scan transformed_scan(scan1.rows(), 2);
	MatrixXi gridIndicesBuf(scan1.rows(), 2);
	double invGridSize = 1.0 / grid_size;

	double a = angle_min;
	double b = angle_max;
	double h = b - a;

	// Initial interior points
	double c = a + invphi2 * h;
	double d = a + invphi * h;

	// Evaluate at initial interior points
	transformScanInPlace(transformed_scan, scan1, tx, ty, c);
	double fc = computeNDTScore(transformed_scan, ndt_grid, invGridSize, gridIndicesBuf);

	transformScanInPlace(transformed_scan, scan1, tx, ty, d);
	double fd = computeNDTScore(transformed_scan, ndt_grid, invGridSize, gridIndicesBuf);

	// Iterate until convergence
	while (h > tol) {
		if (fc < fd) {
			// Minimum is in [a, d]
			b = d;
			d = c;
			fd = fc;
			h = b - a;
			c = a + invphi2 * h;
			transformScanInPlace(transformed_scan, scan1, tx, ty, c);
			fc = computeNDTScore(transformed_scan, ndt_grid, invGridSize, gridIndicesBuf);
		} else {
			// Minimum is in [c, b]
			a = c;
			c = d;
			fc = fd;
			h = b - a;
			d = a + invphi * h;
			transformScanInPlace(transformed_scan, scan1, tx, ty, d);
			fd = computeNDTScore(transformed_scan, ndt_grid, invGridSize, gridIndicesBuf);
		}
	}

	// Return midpoint of final bracket
	return (a + b) * 0.5;
}


NDTGrid NDTScanMatcher::computeNDTGrid(const Scan& scan, double gridSize) {
	NDTGrid ndtGrid;
	double invGridSize = 1.0 / gridSize;
	int nPoints = scan.rows();

	if (nPoints == 0) return ndtGrid;

	// Compute grid indices and find bounds
	MatrixXi gridIndices(nPoints, 2);
	int gi = static_cast<int>(floor(scan(0, 0) * invGridSize));
	int gj = static_cast<int>(floor(scan(0, 1) * invGridSize));
	gridIndices(0, 0) = gi;
	gridIndices(0, 1) = gj;
	int minI = gi, maxI = gi, minJ = gj, maxJ = gj;

	for (int i = 1; i < nPoints; ++i) {
		gi = static_cast<int>(floor(scan(i, 0) * invGridSize));
		gj = static_cast<int>(floor(scan(i, 1) * invGridSize));
		gridIndices(i, 0) = gi;
		gridIndices(i, 1) = gj;
		if (gi < minI) minI = gi;
		if (gi > maxI) maxI = gi;
		if (gj < minJ) minJ = gj;
		if (gj > maxJ) maxJ = gj;
	}

	int width = maxI - minI + 1;
	int height = maxJ - minJ + 1;
	ndtGrid.min_i = minI;
	ndtGrid.min_j = minJ;
	ndtGrid.width = width;
	ndtGrid.height = height;
	ndtGrid.cells.resize(width * height);
	ndtGrid.valid.resize(width * height, 0);

	// Group points by grid cell
	std::vector<std::vector<int>> cellPoints(width * height);
	for (int i = 0; i < nPoints; ++i) {
		int ci = gridIndices(i, 0) - minI;
		int cj = gridIndices(i, 1) - minJ;
		cellPoints[ci * height + cj].push_back(i);
	}

	// Compute Gaussian distributions for each cell
	for (int idx = 0; idx < width * height; ++idx) {
		const vector<int>& pointIndices = cellPoints[idx];

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

			ndtGrid.cells[idx] = GaussianCell(mean, covariance);
			ndtGrid.valid[idx] = 1;
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

double NDTScanMatcher::computeNDTScore(const Scan& scan, const NDTGrid& ndtGrid,
                                       double invGridSize, MatrixXi& gridIndices) {
	auto start = std::chrono::high_resolution_clock::now();
	int nPoints = scan.rows();
	double totalScore = nPoints * D3;

		double vx;
		double vy;
	// Compute grid indices in place
	for (int i = 0; i < nPoints; ++i) {
		gridIndices(i, 0) = static_cast<int>(floor(scan(i, 0) * invGridSize));
		gridIndices(i, 1) = static_cast<int>(floor(scan(i, 1) * invGridSize));
		const GaussianCell* cell = ndtGrid.find(gridIndices(i,0), gridIndices(i,1));

		if (cell != NULL) {
			//Vector2d point = scan.row(i);
			//Vector2d v = point - cell->mean;
			vx = scan(i, 0) - cell->mean(0);
			vy = scan(i, 1) - cell->mean(1);

			// Compute Mahalanobis distance efficiently
			double d = (vx*vx*cell->covariance(1,1) +
					   vy*vy*cell->covariance(0,0) -
					   2*cell->covariance(0,1)*vx*vy) * cell->invdet;

			totalScore -= D1 * exp(NEG_HALF_D2 * d);
		}
	}

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// printf("ScoreTime: "); printf("%d",duration.count());printf(" ms\n");
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
	
	double e = -D1 * D2 / 2 * exp(NEG_HALF_D2 * d);
	double score = D3 - D1 * exp(NEG_HALF_D2 * d);
	
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
	/* if (debug) std::cout << "Initial angle: "  << phiInit << "\n"; */

	// Compute NDT grid for the reference scan
	NDTGrid ndtGrid = computeNDTGrid(scan2, gridSize);

	// Initialize parameters
	double tx = txInit, ty = tyInit, phi = phiInit;

	// auto start = std::chrono::high_resolution_clock::now();
	// auto iterates = neldermead(scan2, scan1, gridSize, ndtGrid, 0.0, 0.0, phi);
	// if (!iterates.empty()) {
	// 	auto last_iterate = iterates.back();
	// 	phi = last_iterate[0]; // or last_iterate[1], whichever is better
	// 	std::cout << "NM result:" << phi << "\n";
	// }

	// auto end = std::chrono::high_resolution_clock::now();
	// auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// printf("NeldMTime: "); printf("%d",duration.count());printf(" ms\n");

	/* if (debug) Timer timer; */
	phi = goldenSectionAngleSearch(scan1, ndtGrid, gridSize, 0.0, 0.0, phiInit-0.5,  phiInit+0.5,0.01);
	/* if (debug) timer.mark("goldensection"); */

	/* if (debug) cout << "GS result: " << phi << "\n"; */
	Matrix3d tempSquare = Matrix3d::Zero();
	double prevScore2 = 0.0;
	Matrix3d Amat = Matrix3d::Zero();
	Vector3d bvec = Vector3d::Zero();

	// Pre-allocate matrices to avoid allocations in hot loop
	int nPoints = scan1.rows();
	Scan transformedScan(nPoints, 2);
	Scan transformedScanTest(nPoints, 2);
	MatrixXi gridIndices(nPoints, 2);
	double invGridSize = 1.0 / gridSize;

	for (int count = 0; count < maxIters; ++count) {
		transformScanInPlace(transformedScan, scan1, tx, ty, phi);
		double cphi = cos(phi);
		double sphi = sin(phi);
		Amat.setZero();
		bvec.setZero();

		double score2 = 0.0;

		// Compute grid indices in place
		for (int i = 0; i < nPoints; ++i) {
			gridIndices(i, 0) = static_cast<int>(floor(transformedScan(i, 0) * invGridSize));
			gridIndices(i, 1) = static_cast<int>(floor(transformedScan(i, 1) * invGridSize));
		}

		for (int i = 0; i < nPoints; ++i) {
			Vector2d point = transformedScan.row(i);
			const GaussianCell* cell = ndtGrid.find(gridIndices(i,0), gridIndices(i,1));

			if (cell != NULL) {
				score2 += getNewtonData(Amat, bvec, phi,cphi,sphi, tempSquare,
									  point, *cell);
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

		double score3 = computeNDTScore(transformedScan, ndtGrid, invGridSize, gridIndices);
		transformScanInPlace(transformedScanTest, scan1, tx - alpha*gradTx,
							 ty - alpha*gradTy, phi - alpha*gradPhi);
		double scoreNext = computeNDTScore(transformedScanTest, ndtGrid, invGridSize, gridIndices);

		double lambda2 = bvec.transpose() * sol;
		bool flag = scoreNext > score3 - b*alpha*lambda2;

		while (flag && alpha > 1e-8) {
			alpha *= c;
			transformScanInPlace(transformedScanTest, scan1, tx - alpha*gradTx,
								 ty - alpha*gradTy, phi - alpha*gradPhi);
			scoreNext = computeNDTScore(transformedScanTest, ndtGrid, invGridSize, gridIndices);
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
const double NDTScanMatcher::NEG_HALF_D2 = -0.5 * 0.5643202489410892;
const double NDTScanMatcher::MAX_GRADIENT_NORM = 0.2;
const double NDTScanMatcher::CONVERGENCE_TOL_LAMBDA = 0.01;
const double NDTScanMatcher::CONVERGENCE_TOL_GRAD = 1e-6;
const double NDTScanMatcher::CONVERGENCE_TOL_SCORE = 1e-2;

