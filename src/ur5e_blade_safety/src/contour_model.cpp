#include "ur5e_blade_safety/contour_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace ur5e_blade_safety
{
namespace
{

double norm2(const Vec2 & value)
{
  return std::sqrt(value.x * value.x + value.y * value.y);
}

Vec2 normalize2(const Vec2 & value)
{
  const double length = norm2(value);
  if (length < 1.0e-12) {
    throw std::runtime_error("Cannot normalize a near-zero 2D vector.");
  }
  return Vec2{value.x / length, value.y / length};
}

double dot2(const Vec2 & a, const Vec2 & b)
{
  return a.x * b.x + a.y * b.y;
}

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

bool parseCsvLine(const std::string & line, Vec3 & point)
{
  std::stringstream stream(line);
  std::string item;
  std::vector<double> values;

  while (std::getline(stream, item, ',')) {
    try {
      values.push_back(std::stod(trim(item)));
    } catch (const std::exception &) {
      return false;
    }
  }

  if (values.size() < 2) {
    return false;
  }

  point.x = values[0];
  point.y = values[1];
  point.z = values.size() >= 3 ? values[2] : 0.0;
  return true;
}

}  // namespace

std::string resolvePackagePath(const std::string & path)
{
  if (path.empty() || path.front() == '/') {
    return path;
  }

  const std::string package_share =
    ament_index_cpp::get_package_share_directory("ur5e_blade_safety");
  return package_share + "/" + path;
}

std::vector<Vec3> loadContourCsv(const std::string & path, const double scale)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open contour CSV: " + path);
  }

  std::vector<Vec3> points;
  std::string line;
  while (std::getline(file, line)) {
    if (trim(line).empty()) {
      continue;
    }

    Vec3 point{};
    if (!parseCsvLine(line, point)) {
      continue;
    }

    points.push_back(Vec3{point.x * scale, point.y * scale, point.z * scale});
  }

  if (points.size() < 3) {
    throw std::runtime_error("Contour CSV must contain at least three numeric points.");
  }

  return points;
}

void forceContourZ(std::vector<Vec3> & points, const double z)
{
  for (auto & point : points) {
    point.z = z;
  }
}

double distance3(const Vec3 & a, const Vec3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::size_t findLargestGap(const std::vector<Vec3> & points)
{
  double max_distance = -std::numeric_limits<double>::infinity();
  std::size_t gap_index = 0;

  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t j = (i + 1) % points.size();
    const double d = distance3(points[i], points[j]);
    if (d > max_distance) {
      max_distance = d;
      gap_index = i;
    }
  }

  return gap_index;
}

std::vector<Segment> makeWallSegments(
  const std::vector<Vec3> & points,
  const std::size_t gap_index)
{
  std::vector<Segment> segments;
  segments.reserve(points.size() - 1);

  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i == gap_index) {
      continue;
    }

    const std::size_t j = (i + 1) % points.size();
    segments.push_back(Segment{points[i], points[j]});
  }

  return segments;
}

ContourModel loadFixedContourModel(
  const std::string & contour_csv,
  const double csv_scale,
  const double contour_z)
{
  ContourModel model;
  model.points = loadContourCsv(resolvePackagePath(contour_csv), csv_scale);
  forceContourZ(model.points, contour_z);
  model.opening_gap_index = findLargestGap(model.points);
  model.wall_segments = makeWallSegments(model.points, model.opening_gap_index);
  return model;
}

OpeningAlignmentPlan makeOpeningAlignmentPlan(
  const ContourModel & contour,
  const double pre_offset,
  const double approach_offset,
  const double target_z,
  const std::string & blade_shaft_axis)
{
  const auto & points = contour.points;
  const auto gap_index = contour.opening_gap_index;
  const Vec3 a = points[gap_index];
  const Vec3 b = points[(gap_index + 1) % points.size()];
  const Vec3 center{
    0.5 * (a.x + b.x),
    0.5 * (a.y + b.y),
    0.5 * (a.z + b.z)};

  const Vec2 tangent = normalize2(Vec2{b.x - a.x, b.y - a.y});
  const Vec2 n1{-tangent.y, tangent.x};
  const Vec2 n2{tangent.y, -tangent.x};
  const Vec2 v_origin = normalize2(Vec2{-center.x, -center.y});
  const Vec2 outward = dot2(n1, v_origin) > dot2(n2, v_origin) ? n1 : n2;
  const Vec2 insert{-outward.x, -outward.y};

  const Vec3 pre_center{
    center.x + outward.x * pre_offset,
    center.y + outward.y * pre_offset,
    target_z};
  const Vec3 approach_center{
    pre_center.x + outward.x * approach_offset,
    pre_center.y + outward.y * approach_offset,
    target_z};

  double yaw = std::atan2(insert.y, insert.x);
  if (blade_shaft_axis == "minus_y") {
    yaw += M_PI / 2.0;
  } else if (blade_shaft_axis == "plus_y") {
    yaw -= M_PI / 2.0;
  } else {
    throw std::runtime_error(
            "blade_shaft_axis must be either 'minus_y' or 'plus_y', got: " + blade_shaft_axis);
  }

  return OpeningAlignmentPlan{a, b, center, outward, insert, approach_center, pre_center, yaw};
}

DistanceResult distancePointToSegment2d(const Vec3 & point, const Segment & segment)
{
  const double ax = segment.a.x;
  const double ay = segment.a.y;
  const double bx = segment.b.x;
  const double by = segment.b.y;
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = point.x - ax;
  const double apy = point.y - ay;
  const double ab_len2 = abx * abx + aby * aby;

  double u = 0.0;
  if (ab_len2 > 1.0e-12) {
    u = std::clamp((apx * abx + apy * aby) / ab_len2, 0.0, 1.0);
  }

  const Vec3 closest{ax + u * abx, ay + u * aby, segment.a.z + u * (segment.b.z - segment.a.z)};
  const double dx = point.x - closest.x;
  const double dy = point.y - closest.y;
  return DistanceResult{std::sqrt(dx * dx + dy * dy), point, closest};
}

DistanceResult computeMinimumDistance(
  const std::vector<Vec3> & samples,
  const std::vector<Segment> & segments)
{
  DistanceResult best{
    std::numeric_limits<double>::infinity(),
    Vec3{0.0, 0.0, 0.0},
    Vec3{0.0, 0.0, 0.0}};

  for (const auto & sample : samples) {
    for (const auto & segment : segments) {
      const DistanceResult current = distancePointToSegment2d(sample, segment);
      if (current.distance < best.distance) {
        best = current;
      }
    }
  }

  return best;
}

}  // namespace ur5e_blade_safety
