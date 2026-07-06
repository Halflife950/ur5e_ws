#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/logger.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace ur5e_curve_path
{

struct PathPoint
{
  double x_mm{};
  double y_mm{};
  double z_mm{};
};

enum class CsvAxis
{
  X,
  Y,
  Z,
  Fixed
};

struct PathAxisMapping
{
  CsvAxis base_x_from_csv_axis{CsvAxis::X};
  CsvAxis base_y_from_csv_axis{CsvAxis::Y};
  CsvAxis base_z_from_csv_axis{CsvAxis::Fixed};
};

struct PathDimensions
{
  double x_mm{};
  double y_mm{};
  double z_mm{};
};

struct PathBounds
{
  PathDimensions raw_dimensions;
  PathDimensions mapped_dimensions;
};

double pointDistanceMm(
  const PathPoint & first,
  const PathPoint & second);

CsvAxis parseCsvAxis(
  const std::string & value,
  bool allow_fixed,
  const std::string & parameter_name);

std::string csvAxisName(
  CsvAxis axis);

double csvAxisOffsetMeters(
  const PathPoint & point,
  const PathPoint & origin,
  CsvAxis axis);

geometry_msgs::msg::Point transformPathPoint(
  const PathPoint & point,
  const PathPoint & origin_point,
  const geometry_msgs::msg::Point & path_origin,
  const PathAxisMapping & axis_mapping);

double pointDistanceMeters(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second);

PathBounds calculatePathBounds(
  const std::vector<PathPoint> & points,
  const PathAxisMapping & axis_mapping);

std::vector<PathPoint> loadCsv(
  const std::string & file_path);

void validatePathContinuity(
  const std::vector<PathPoint> & points,
  double max_gap_mm,
  const rclcpp::Logger & logger);

}  // namespace ur5e_curve_path
