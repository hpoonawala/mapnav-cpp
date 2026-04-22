#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Dense>
#include "frameHistory.h"

static const double EPS = 1e-9;
static const double PI = M_PI;

// Helper: build a single-row scan from one (x,y) point.
static Scan makePoint(double x, double y) {
    Scan s(1, 2);
    s << x, y;
    return s;
}

// ---- transformScan ------------------------------------------------------

TEST(TransformScan, IdentityLeavesPointUnchanged) {
    Scan result = transformScan(makePoint(2.0, 3.0), 0.0, 0.0, 0.0);
    EXPECT_NEAR(result(0, 0), 2.0, EPS);
    EXPECT_NEAR(result(0, 1), 3.0, EPS);
}

TEST(TransformScan, PureTranslation) {
    Scan result = transformScan(makePoint(1.0, 2.0), 3.0, 4.0, 0.0);
    EXPECT_NEAR(result(0, 0), 4.0, EPS);
    EXPECT_NEAR(result(0, 1), 6.0, EPS);
}

TEST(TransformScan, Rotate90CCW) {
    // (1,0) rotated 90° CCW should become (0,1).
    Scan result = transformScan(makePoint(1.0, 0.0), 0.0, 0.0, PI / 2);
    EXPECT_NEAR(result(0, 0), 0.0, 1e-9);
    EXPECT_NEAR(result(0, 1), 1.0, 1e-9);
}

TEST(TransformScan, Rotate180) {
    // (1,0) rotated 180° should become (-1,0).
    Scan result = transformScan(makePoint(1.0, 0.0), 0.0, 0.0, PI);
    EXPECT_NEAR(result(0, 0), -1.0, 1e-9);
    EXPECT_NEAR(result(0, 1),  0.0, 1e-9);
}

TEST(TransformScan, RotationThenTranslation) {
    // Rotate (1,0) by 90° CCW → (0,1), then translate by (2,3) → (2,4).
    Scan result = transformScan(makePoint(1.0, 0.0), 2.0, 3.0, PI / 2);
    EXPECT_NEAR(result(0, 0), 2.0, 1e-9);
    EXPECT_NEAR(result(0, 1), 4.0, 1e-9);
}

// ---- transformScanInPlace -----------------------------------------------

TEST(TransformScanInPlace, MatchesTransformScanOnMultiplePoints) {
    Scan scan(3, 2);
    scan << 1.0, 0.0,
            0.0, 1.0,
            1.0, 1.0;
    double tx = 0.5, ty = -0.3, phi = PI / 6;

    Scan expected = transformScan(scan, tx, ty, phi);
    Scan output(3, 2);
    transformScanInPlace(output, scan, tx, ty, phi);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(output(i, 0), expected(i, 0), 1e-9);
        EXPECT_NEAR(output(i, 1), expected(i, 1), 1e-9);
    }
}

// ---- transformScanToPose ------------------------------------------------

TEST(TransformScanToPose, EquivalentToTransformScan) {
    Scan scan(2, 2);
    scan << 2.0, 0.0,
            0.0, 2.0;
    Pose2D pose(1.0, 1.0, PI / 4);
    Scan expected = transformScan(scan, pose.x_, pose.y_, pose.theta_);
    Scan result   = transformScanToPose(scan, pose);
    for (int i = 0; i < 2; i++) {
        EXPECT_NEAR(result(i, 0), expected(i, 0), EPS);
        EXPECT_NEAR(result(i, 1), expected(i, 1), EPS);
    }
}
