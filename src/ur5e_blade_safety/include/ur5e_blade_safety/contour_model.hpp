#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace ur5e_blade_safety
{

struct Vec2
{
  double x;
  double y;
};

struct Vec3
{
  double x;
  double y;
  double z;
};

struct Segment
{
  Vec3 a;
  Vec3 b;
};

struct DistanceResult
{
  double distance;
  Vec3 sample_point;
  Vec3 closest_boundary_point;
};

struct OpeningAlignmentPlan
{
  Vec3 opening_a;
  Vec3 opening_b;
  Vec3 opening_center;
  Vec2 outward_dir;
  Vec2 insert_dir;
  Vec3 approach_center;
  Vec3 pre_center;
  double yaw;
};

struct ContourModel
{
  std::vector<Vec3> points;
  std::vector<Segment> wall_segments;
  std::size_t opening_gap_index;
};

std::string resolvePackagePath(const std::string & path);

std::vector<Vec3> loadContourCsv(const std::string & path, double scale);

void forceContourZ(std::vector<Vec3> & points, double z);

double distance3(const Vec3 & a, const Vec3 & b);

std::size_t findLargestGap(const std::vector<Vec3> & points);

std::vector<Segment> makeWallSegments(
  const std::vector<Vec3> & points,
  std::size_t gap_index);

ContourModel loadFixedContourModel(
  const std::string & contour_csv,
  double csv_scale,
  double contour_z);

OpeningAlignmentPlan makeOpeningAlignmentPlan(
  const ContourModel & contour,
  double pre_offset,
  double approach_offset,
  double target_z,
  const std::string & blade_shaft_axis);

DistanceResult distancePointToSegment2d(const Vec3 & point, const Segment & segment);

DistanceResult computeMinimumDistance(
  const std::vector<Vec3> & samples,
  const std::vector<Segment> & segments);

}  // namespace ur5e_blade_safety
