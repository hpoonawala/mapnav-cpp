#include <gtest/gtest.h>
#include <cmath>
#include "pose.h"

static const double EPS = 1e-9;
static const double PI = M_PI;

// ---- move_pose_local ----------------------------------------------------

TEST(MovePoseLocal, ForwardWhenFacingEast) {
    Pose2D pose(0.0, 0.0, 0.0);
    move_pose_local(pose, Pose2D(1.0, 0.0, 0.0));
    EXPECT_NEAR(pose.x_, 1.0, EPS);
    EXPECT_NEAR(pose.y_, 0.0, EPS);
    EXPECT_NEAR(pose.theta_, 0.0, EPS);
}

TEST(MovePoseLocal, ForwardWhenFacingNorth) {
    // Facing PI/2 (north), moving forward should increase y, not x.
    Pose2D pose(0.0, 0.0, PI / 2);
    move_pose_local(pose, Pose2D(1.0, 0.0, 0.0));
    EXPECT_NEAR(pose.x_, 0.0, EPS);
    EXPECT_NEAR(pose.y_, 1.0, EPS);
}

TEST(MovePoseLocal, SidewaysWhenFacingEast) {
    // Local +y is left (north) when facing east.
    Pose2D pose(0.0, 0.0, 0.0);
    move_pose_local(pose, Pose2D(0.0, 1.0, 0.0));
    EXPECT_NEAR(pose.x_, 0.0, EPS);
    EXPECT_NEAR(pose.y_, 1.0, EPS);
}

TEST(MovePoseLocal, RotationAccumulates) {
    Pose2D pose(0.0, 0.0, 0.0);
    move_pose_local(pose, Pose2D(0.0, 0.0, PI / 4));
    move_pose_local(pose, Pose2D(0.0, 0.0, PI / 4));
    EXPECT_NEAR(pose.theta_, PI / 2, EPS);
}

// ---- invert_pose --------------------------------------------------------

TEST(InvertPose, PureTranslationNoRotation) {
    // Inverse of a pure translation is the negative translation.
    Pose2D pose(3.0, 4.0, 0.0);
    Pose2D inv = invert_pose(pose);
    EXPECT_NEAR(inv.x_, -3.0, EPS);
    EXPECT_NEAR(inv.y_, -4.0, EPS);
    EXPECT_NEAR(inv.theta_, 0.0, EPS);
}

TEST(InvertPose, ComposeWithInverseGivesIdentity) {
    // Applying a pose then its inverse should return to origin.
    Pose2D p(2.0, 1.0, PI / 3);
    Pose2D at_origin(0.0, 0.0, 0.0);
    move_pose_local(at_origin, p);
    move_pose_local(at_origin, invert_pose(p));
    EXPECT_NEAR(at_origin.x_, 0.0, 1e-9);
    EXPECT_NEAR(at_origin.y_, 0.0, 1e-9);
    EXPECT_NEAR(at_origin.theta_, 0.0, 1e-9);
}

// ---- normalizeAngle -----------------------------------------------------

TEST(NormalizeAngle, InRangeUnchanged) {
    EXPECT_NEAR(Pose2D::normalizeAngle(0.0),        0.0,        EPS);
    EXPECT_NEAR(Pose2D::normalizeAngle(PI - 0.1),   PI - 0.1,   EPS);
    EXPECT_NEAR(Pose2D::normalizeAngle(-PI + 0.1),  -PI + 0.1,  EPS);
}

TEST(NormalizeAngle, LargePositiveWraps) {
    // 3π is one full rotation past π, so should normalise to π (or -π).
    double result = Pose2D::normalizeAngle(3.0 * PI);
    EXPECT_NEAR(std::abs(result), PI, EPS);
}

TEST(NormalizeAngle, LargeNegativeWraps) {
    double result = Pose2D::normalizeAngle(-3.0 * PI);
    EXPECT_NEAR(std::abs(result), PI, EPS);
}

// ---- distanceTo ---------------------------------------------------------

TEST(Pose2DDistance, Pythagorean345) {
    Pose2D a(0.0, 0.0, 0.0);
    Pose2D b(3.0, 4.0, 0.0);
    EXPECT_NEAR(a.distanceTo(b), 5.0, EPS);
}

TEST(Pose2DDistance, Symmetric) {
    Pose2D a(1.0, 2.0, 0.0);
    Pose2D b(4.0, 6.0, 0.0);
    EXPECT_NEAR(a.distanceTo(b), b.distanceTo(a), EPS);
}

TEST(Pose2DDistance, SamePointIsZero) {
    Pose2D a(1.5, -3.0, 0.7);
    EXPECT_NEAR(a.distanceTo(a), 0.0, EPS);
}

// ---- relative_pose_to ---------------------------------------------------

TEST(RelativePoseTo, BaseAtOriginIsIdentity) {
    // If base is at origin with no rotation, relative == world.
    Pose2D world(1.0, 2.0, 0.5);
    Pose2D base(0.0, 0.0, 0.0);
    Pose2D rel;
    world.relative_pose_to(base, rel);
    EXPECT_NEAR(rel.x_,     1.0, EPS);
    EXPECT_NEAR(rel.y_,     2.0, EPS);
    EXPECT_NEAR(rel.theta_, 0.5, EPS);
}

TEST(RelativePoseTo, BaseRotated90) {
    // Base faces north (PI/2). World point is 1m east of base.
    // In base's local frame that point is directly to the right (local y = -1).
    Pose2D world(1.0, 0.0, 0.0);
    Pose2D base(0.0, 0.0, PI / 2);
    Pose2D rel;
    world.relative_pose_to(base, rel);
    EXPECT_NEAR(rel.x_, 0.0,  EPS);
    EXPECT_NEAR(rel.y_, -1.0, EPS);
}
