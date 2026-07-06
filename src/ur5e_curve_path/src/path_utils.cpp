#include "ur5e_curve_path/path_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ur5e_curve_path
{
namespace
{
constexpr double kMmToM = 0.001;

std::string trim(
  const std::string & value)
{
  const auto first =
    value.find_first_not_of(" \t\r\n");

  if (first == std::string::npos)
  {
    return "";
  }

  const auto last =
    value.find_last_not_of(" \t\r\n");

  return value.substr(
    first,
    last - first + 1);
}


double parseCsvDouble(
  const std::string & value,
  const std::string & field_name)
{
  const std::string cleaned_value =
    trim(value);

  std::size_t parsed_length = 0;

  const double parsed_value =
    std::stod(
      cleaned_value,
      &parsed_length);

  if (parsed_length != cleaned_value.size())
  {
    throw std::runtime_error(
      "Unexpected trailing characters in " +
      field_name +
      " value.");
  }

  if (!std::isfinite(parsed_value))
  {
    throw std::runtime_error(
      field_name +
      " value is not finite.");
  }

  return parsed_value;
}

}  // namespace


double pointDistanceMm(
  const PathPoint & first,
  const PathPoint & second)
{
  const double dx = second.x_mm - first.x_mm;
  const double dy = second.y_mm - first.y_mm;
  const double dz = second.z_mm - first.z_mm;

  return std::sqrt(
    dx * dx +
    dy * dy +
    dz * dz);
}


CsvAxis parseCsvAxis(
  const std::string & value,
  const bool allow_fixed,
  const std::string & parameter_name)
{
  const std::string axis =
    trim(value);

  if (axis == "x")
  {
    return CsvAxis::X;
  }

  if (axis == "y")
  {
    return CsvAxis::Y;
  }

  if (axis == "z")
  {
    return CsvAxis::Z;
  }

  if (
    allow_fixed &&
    axis == "fixed")
  {
    return CsvAxis::Fixed;
  }

  throw std::runtime_error(
    "Parameter " +
    parameter_name +
    (
      allow_fixed ?
      " must be fixed, x, y, or z." :
      " must be x, y, or z."
    ));
}


std::string csvAxisName(
  const CsvAxis axis)
{
  switch (axis)
  {
    case CsvAxis::X:
      return "x";
    case CsvAxis::Y:
      return "y";
    case CsvAxis::Z:
      return "z";
    case CsvAxis::Fixed:
      return "fixed";
  }

  throw std::runtime_error(
    "Unsupported CSV axis.");
}


double csvAxisOffsetMeters(
  const PathPoint & point,
  const PathPoint & origin,
  const CsvAxis axis)
{
  switch (axis)
  {
    case CsvAxis::X:
      return
        (point.x_mm - origin.x_mm) *
        kMmToM;
    case CsvAxis::Y:
      return
        (point.y_mm - origin.y_mm) *
        kMmToM;
    case CsvAxis::Z:
      return
        (point.z_mm - origin.z_mm) *
        kMmToM;
    case CsvAxis::Fixed:
      return 0.0;
  }

  throw std::runtime_error(
    "Unsupported CSV axis.");
}


geometry_msgs::msg::Point transformPathPoint(
  const PathPoint & point,
  const PathPoint & origin_point,
  const geometry_msgs::msg::Point & path_origin,
  const PathAxisMapping & axis_mapping)
{
  geometry_msgs::msg::Point target;

  target.x =
    path_origin.x +
    csvAxisOffsetMeters(
      point,
      origin_point,
      axis_mapping.base_x_from_csv_axis);

  target.y =
    path_origin.y +
    csvAxisOffsetMeters(
      point,
      origin_point,
      axis_mapping.base_y_from_csv_axis);

  target.z =
    path_origin.z +
    csvAxisOffsetMeters(
      point,
      origin_point,
      axis_mapping.base_z_from_csv_axis);

  return target;
}


double pointDistanceMeters(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  const double dx =
    second.x -
    first.x;

  const double dy =
    second.y -
    first.y;

  const double dz =
    second.z -
    first.z;

  return std::sqrt(
    dx * dx +
    dy * dy +
    dz * dz);
}


PathBounds calculatePathBounds(
  const std::vector<PathPoint> & points,
  const PathAxisMapping & axis_mapping)
{
  if (points.empty())
  {
    throw std::runtime_error(
      "Unable to calculate bounds for an empty path.");
  }

  double min_x = points.front().x_mm;
  double max_x = points.front().x_mm;

  double min_y = points.front().y_mm;
  double max_y = points.front().y_mm;

  double min_z = points.front().z_mm;
  double max_z = points.front().z_mm;

  const PathPoint & origin_point =
    points.front();

  double min_base_x = 0.0;
  double max_base_x = 0.0;

  double min_base_y = 0.0;
  double max_base_y = 0.0;

  double min_base_z = 0.0;
  double max_base_z = 0.0;

  for (const auto & point : points)
  {
    min_x = std::min(min_x, point.x_mm);
    max_x = std::max(max_x, point.x_mm);

    min_y = std::min(min_y, point.y_mm);
    max_y = std::max(max_y, point.y_mm);

    min_z = std::min(min_z, point.z_mm);
    max_z = std::max(max_z, point.z_mm);

    const double base_x =
      csvAxisOffsetMeters(
        point,
        origin_point,
        axis_mapping.base_x_from_csv_axis);

    const double base_y =
      csvAxisOffsetMeters(
        point,
        origin_point,
        axis_mapping.base_y_from_csv_axis);

    const double base_z =
      csvAxisOffsetMeters(
        point,
        origin_point,
        axis_mapping.base_z_from_csv_axis);

    min_base_x =
      std::min(
        min_base_x,
        base_x);

    max_base_x =
      std::max(
        max_base_x,
        base_x);

    min_base_y =
      std::min(
        min_base_y,
        base_y);

    max_base_y =
      std::max(
        max_base_y,
        base_y);

    min_base_z =
      std::min(
        min_base_z,
        base_z);

    max_base_z =
      std::max(
        max_base_z,
        base_z);
  }

  PathBounds bounds;

  bounds.raw_dimensions.x_mm =
    max_x -
    min_x;
  bounds.raw_dimensions.y_mm =
    max_y -
    min_y;
  bounds.raw_dimensions.z_mm =
    max_z -
    min_z;

  bounds.mapped_dimensions.x_mm =
    (
      max_base_x -
      min_base_x
    ) *
    1000.0;
  bounds.mapped_dimensions.y_mm =
    (
      max_base_y -
      min_base_y
    ) *
    1000.0;
  bounds.mapped_dimensions.z_mm =
    (
      max_base_z -
      min_base_z
    ) *
    1000.0;

  return bounds;
}


std::vector<PathPoint> loadCsv(
  const std::string & file_path)
{
  std::ifstream file(file_path);

  if (!file.is_open())
  {
    throw std::runtime_error(
      "Unable to open CSV file: " + file_path);
  }

  std::vector<PathPoint> points;
  std::string line;

  // Skip the CSV header.
  if (!std::getline(file, line))
  {
    throw std::runtime_error(
      "The CSV file is empty: " + file_path);
  }

  std::size_t line_number = 1;

  while (std::getline(file, line))
  {
    ++line_number;

    if (line.empty())
    {
      continue;
    }

    std::stringstream stream(line);
    std::string value;
    PathPoint point;

    try
    {
      if (!std::getline(stream, value, ','))
      {
        throw std::runtime_error("Missing x value.");
      }
      point.x_mm =
        parseCsvDouble(
        value,
        "x");

      if (!std::getline(stream, value, ','))
      {
        throw std::runtime_error("Missing y value.");
      }
      point.y_mm =
        parseCsvDouble(
        value,
        "y");

      if (!std::getline(stream, value, ','))
      {
        throw std::runtime_error("Missing z value.");
      }
      point.z_mm =
        parseCsvDouble(
        value,
        "z");

      if (std::getline(stream, value, ','))
      {
        throw std::runtime_error(
          "Unexpected extra CSV column.");
      }
    }
    catch (const std::exception & error)
    {
      throw std::runtime_error(
        "Invalid CSV data at line " +
        std::to_string(line_number) +
        ": " +
        error.what());
    }

    points.push_back(point);
  }

  if (points.size() < 2)
  {
    throw std::runtime_error(
      "The path must contain at least two points.");
  }

  return points;
}


void validatePathContinuity(
  const std::vector<PathPoint> & points,
  const double max_gap_mm,
  const rclcpp::Logger & logger)
{
  double largest_adjacent_gap_mm = 0.0;
  std::size_t largest_gap_index = 0;

  for (std::size_t i = 0; i + 1 < points.size(); ++i)
  {
    const double gap_mm =
      pointDistanceMm(points[i], points[i + 1]);

    if (gap_mm > largest_adjacent_gap_mm)
    {
      largest_adjacent_gap_mm = gap_mm;
      largest_gap_index = i;
    }

    if (gap_mm > max_gap_mm)
    {
      throw std::runtime_error(
        "Path continuity check failed between points " +
        std::to_string(i) +
        " and " +
        std::to_string(i + 1) +
        ". Gap: " +
        std::to_string(gap_mm) +
        " mm. Maximum allowed gap: " +
        std::to_string(max_gap_mm) +
        " mm.");
    }
  }

  const double closing_gap_mm =
    pointDistanceMm(points.back(), points.front());

  if (closing_gap_mm > max_gap_mm)
  {
    throw std::runtime_error(
      "Path closure check failed between the last and first points. "
      "Gap: " +
      std::to_string(closing_gap_mm) +
      " mm. Maximum allowed gap: " +
      std::to_string(max_gap_mm) +
      " mm.");
  }

  RCLCPP_INFO(
    logger,
    "Path continuity check passed.");

  RCLCPP_INFO(
    logger,
    "Largest adjacent gap: %.3f mm between points %zu and %zu.",
    largest_adjacent_gap_mm,
    largest_gap_index,
    largest_gap_index + 1);

  RCLCPP_INFO(
    logger,
    "Closing gap: %.3f mm.",
    closing_gap_mm);
}

}  // namespace ur5e_curve_path
