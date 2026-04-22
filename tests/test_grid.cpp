#include <gtest/gtest.h>
#include "OccupancyGrid.h"

// 10 x 10 metre grid at 2 cm resolution → 500 x 500 cells.
static const double W   = 10.0;
static const double H   = 10.0;
static const double RES = 0.02;

class GridCoordTest : public ::testing::Test {
protected:
    OccupancyGrid grid{W, H, RES};
};

// ---- worldToGrid / gridToWorld ------------------------------------------

TEST_F(GridCoordTest, OriginMapsToCenter) {
    std::pair<int,int> ij = grid.worldToGrid(0.0, 0.0);
    std::pair<int,int> sz = grid.getGridSize();
    EXPECT_EQ(ij.first,  sz.first  / 2);
    EXPECT_EQ(ij.second, sz.second / 2);
}

TEST_F(GridCoordTest, RoundTripOrigin) {
    // worldToGrid then gridToWorld should return a point within one cell of origin.
    std::pair<int,int> ij = grid.worldToGrid(0.0, 0.0);
    std::pair<double,double> xy = grid.gridToWorld(ij.first, ij.second);
    EXPECT_NEAR(xy.first,  0.0, RES);
    EXPECT_NEAR(xy.second, 0.0, RES);
}

TEST_F(GridCoordTest, RoundTripArbitraryPoint) {
    // For any in-bounds world point, the round-trip is within one resolution cell.
    double wx = 1.5, wy = -2.3;
    std::pair<int,int> ij = grid.worldToGrid(wx, wy);
    std::pair<double,double> xy = grid.gridToWorld(ij.first, ij.second);
    EXPECT_NEAR(xy.first,  wx, RES);
    EXPECT_NEAR(xy.second, wy, RES);
}

TEST_F(GridCoordTest, CornerPointsMapToEdgeCells) {
    // The far positive corner should map near the max cell index.
    std::pair<int,int> ij = grid.worldToGrid(4.9, 4.9);
    std::pair<int,int> sz = grid.getGridSize();
    EXPECT_GT(ij.first,  sz.first  * 9 / 10);
    EXPECT_GT(ij.second, sz.second * 9 / 10);
}

// ---- isValidWorldCoord --------------------------------------------------

TEST_F(GridCoordTest, OriginIsValid) {
    EXPECT_TRUE(grid.isValidWorldCoord(0.0, 0.0));
}

TEST_F(GridCoordTest, InBoundsPointIsValid) {
    EXPECT_TRUE(grid.isValidWorldCoord(3.0, -2.5));
}

TEST_F(GridCoordTest, FarPointIsInvalid) {
    EXPECT_FALSE(grid.isValidWorldCoord(100.0, 0.0));
}

TEST_F(GridCoordTest, BoundaryIsInvalid) {
    // Boundary is exclusive at +W/2, +H/2.
    EXPECT_FALSE(grid.isValidWorldCoord(5.0, 0.0));
    EXPECT_FALSE(grid.isValidWorldCoord(0.0, 5.0));
}

// ---- log-odds set / get / clear -----------------------------------------

TEST_F(GridCoordTest, FreshGridIsAllUnknown) {
    // Zero log-odds means unknown (equal probability of free/occupied).
    std::pair<int,int> ij = grid.worldToGrid(0.0, 0.0);
    EXPECT_NEAR(grid.getLogOdds(ij.first, ij.second), 0.0, 1e-9);
}

TEST_F(GridCoordTest, SetAndGetLogOdds) {
    std::pair<int,int> ij = grid.worldToGrid(1.0, 1.0);
    grid.setLogOdds(ij.first, ij.second, 2.5);
    EXPECT_NEAR(grid.getLogOdds(ij.first, ij.second), 2.5, 1e-9);
}

TEST_F(GridCoordTest, ClearResetsToZero) {
    std::pair<int,int> ij = grid.worldToGrid(1.0, 1.0);
    grid.setLogOdds(ij.first, ij.second, 2.5);
    grid.clear();
    EXPECT_NEAR(grid.getLogOdds(ij.first, ij.second), 0.0, 1e-9);
}

TEST_F(GridCoordTest, OutOfBoundsSetReturnsFalse) {
    EXPECT_FALSE(grid.setLogOdds(-1, 0, 1.0));
}
