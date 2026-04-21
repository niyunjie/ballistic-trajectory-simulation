#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgEarth/EarthManipulator>
#include <osgEarth/GeoData>
#include <osgEarth/GLUtils>
#include <osgEarth/MapNode>
#include <osgEarth/PlaceNode>
#include <osgEarth/Registry>
#include <osgEarth/Style>
#include <osgEarth/TextSymbol>
#include <osgEarth/Viewpoint>
#include <osgEarthImGui/ImGuiEventHandler>
#include <osgEarthImGui/imgui.h>
#include <osgUtil/Tessellator>
#include <osgViewer/Viewer>

#include "ballistics/defaults.hpp"
#include "ballistics/dynamics.hpp"
#include "ballistics/logging.hpp"
#include "ballistics/solver.hpp"

namespace {

namespace oe = osgEarth;
namespace oeu = osgEarth::Util;

enum class SolveMode { Farthest, TargetPoint };

struct SeriesBundle {
  struct Entry {
    std::string name;
    std::vector<ballistics::PlotPoint> points;
    ImU32 color = IM_COL32_WHITE;
    float thickness = 2.0f;
  };

  std::vector<Entry> entries;
};

static bool InputDouble(const char* label, double* value, double step = 1.0,
                        const char* format = "%.3f") {
  return ImGui::InputScalar(label, ImGuiDataType_Double, value, &step, nullptr,
                            format);
}

template <typename... Args>
std::string FormatText(const char* format, Args... args) {
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), format, args...);
  return std::string(buffer);
}

constexpr double kPi = 3.14159265358979323846;

static double DegToRad(double degrees) {
  return degrees * kPi / 180.0;
}

static double RadToDeg(double radians) {
  return radians * 180.0 / kPi;
}

struct SvgPoint {
  double x = 0.0;
  double y = 0.0;
};

struct SvgBounds {
  double minX = 0.0;
  double maxX = 1.0;
  double minY = 0.0;
  double maxY = 1.0;
};

static std::string FormatNumber(double value, int precision = 3) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}

static std::string EscapeCsvField(const std::string& value) {
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (char ch : value) {
    if (ch == '"') {
      escaped += "\"\"";
    } else {
      escaped.push_back(ch);
    }
  }
  escaped.push_back('"');
  return escaped;
}

static std::string SlugifyFilename(const std::string& value) {
  std::string slug;
  slug.reserve(value.size());
  bool lastWasDash = false;
  for (char ch : value) {
    const unsigned char uch = static_cast<unsigned char>(ch);
    if ((uch >= 'a' && uch <= 'z') || (uch >= '0' && uch <= '9')) {
      slug.push_back(static_cast<char>(uch));
      lastWasDash = false;
    } else if (uch >= 'A' && uch <= 'Z') {
      slug.push_back(static_cast<char>(uch - 'A' + 'a'));
      lastWasDash = false;
    } else if (!lastWasDash) {
      slug.push_back('-');
      lastWasDash = true;
    }
  }
  while (!slug.empty() && slug.back() == '-') {
    slug.pop_back();
  }
  while (!slug.empty() && slug.front() == '-') {
    slug.erase(slug.begin());
  }
  return slug.empty() ? "method" : slug;
}

static bool WriteTextFile(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  FILE* file = _wfopen(path.c_str(), L"wb");
  if (file == nullptr) {
    return false;
  }
  std::fwrite(text.data(), 1, text.size(), file);
  std::fclose(file);
  return true;
}

static std::string TimestampTag() {
  std::time_t now = std::time(nullptr);
  std::tm localTime{};
  localtime_s(&localTime, &now);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &localTime);
  return std::string(buffer);
}

static SvgBounds ComputeBounds(const std::vector<SvgPoint>& points) {
  SvgBounds bounds;
  if (points.empty()) {
    return bounds;
  }
  bounds.minX = bounds.maxX = points.front().x;
  bounds.minY = bounds.maxY = points.front().y;
  for (const auto& point : points) {
    bounds.minX = std::min(bounds.minX, point.x);
    bounds.maxX = std::max(bounds.maxX, point.x);
    bounds.minY = std::min(bounds.minY, point.y);
    bounds.maxY = std::max(bounds.maxY, point.y);
  }
  if (std::abs(bounds.maxX - bounds.minX) < 1e-9) {
    bounds.maxX = bounds.minX + 1.0;
  }
  if (std::abs(bounds.maxY - bounds.minY) < 1e-9) {
    bounds.maxY = bounds.minY + 1.0;
  }
  return bounds;
}

static std::string BuildPolylineSvg(const std::string& title, const std::string& xLabel,
                                    const std::string& yLabel,
                                    const std::vector<SvgPoint>& points,
                                    const char* strokeColor,
                                    const char* fillColor = "none",
                                    bool closePath = false) {
  constexpr double width = 960.0;
  constexpr double height = 560.0;
  constexpr double left = 90.0;
  constexpr double right = 28.0;
  constexpr double top = 56.0;
  constexpr double bottom = 72.0;
  const SvgBounds bounds = ComputeBounds(points);
  const double plotWidth = width - left - right;
  const double plotHeight = height - top - bottom;

  auto mapX = [&](double value) {
    return left + (value - bounds.minX) / (bounds.maxX - bounds.minX) * plotWidth;
  };
  auto mapY = [&](double value) {
    return top + plotHeight - (value - bounds.minY) / (bounds.maxY - bounds.minY) * plotHeight;
  };

  std::ostringstream svg;
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " "
      << height << "\">";
  svg << "<rect width=\"100%\" height=\"100%\" fill=\"#08111f\"/>";
  svg << "<rect x=\"" << left << "\" y=\"" << top << "\" width=\"" << plotWidth
      << "\" height=\"" << plotHeight
      << "\" fill=\"#0f1b2d\" stroke=\"#35506d\" stroke-width=\"1.5\" rx=\"12\"/>";
  for (int i = 0; i <= 4; ++i) {
    const double y = top + plotHeight * static_cast<double>(i) / 4.0;
    svg << "<line x1=\"" << left << "\" y1=\"" << y << "\" x2=\""
        << left + plotWidth << "\" y2=\"" << y
        << "\" stroke=\"#26415c\" stroke-dasharray=\"6 8\" stroke-width=\"1\"/>";
  }
  for (int i = 0; i <= 5; ++i) {
    const double x = left + plotWidth * static_cast<double>(i) / 5.0;
    svg << "<line x1=\"" << x << "\" y1=\"" << top << "\" x2=\"" << x
        << "\" y2=\"" << top + plotHeight
        << "\" stroke=\"#21374f\" stroke-dasharray=\"6 8\" stroke-width=\"1\"/>";
  }

  svg << "<text x=\"" << width * 0.5
      << "\" y=\"30\" text-anchor=\"middle\" font-size=\"24\" font-family=\"Segoe UI\" fill=\"#f2f6fb\">"
      << title << "</text>";
  svg << "<text x=\"" << width * 0.5 << "\" y=\"" << height - 20
      << "\" text-anchor=\"middle\" font-size=\"18\" font-family=\"Segoe UI\" fill=\"#c7d7e8\">"
      << xLabel << "</text>";
  svg << "<text x=\"26\" y=\"" << height * 0.5
      << "\" text-anchor=\"middle\" transform=\"rotate(-90 26 " << height * 0.5
      << ")\" font-size=\"18\" font-family=\"Segoe UI\" fill=\"#c7d7e8\">"
      << yLabel << "</text>";

  if (!points.empty()) {
    svg << "<polyline fill=\"" << fillColor << "\" stroke=\"" << strokeColor
        << "\" fill-opacity=\""
        << ((std::string(fillColor) == "none") ? "1.0" : "0.16")
        << "\" stroke-width=\"4\" stroke-linejoin=\"round\" stroke-linecap=\"round\" points=\"";
    for (const auto& point : points) {
      svg << FormatNumber(mapX(point.x), 2) << "," << FormatNumber(mapY(point.y), 2)
          << " ";
    }
    if (closePath) {
      svg << FormatNumber(mapX(points.front().x), 2) << ","
          << FormatNumber(mapY(points.front().y), 2);
    }
    svg << "\"/>";
  } else {
    svg << "<text x=\"" << width * 0.5 << "\" y=\"" << height * 0.5
        << "\" text-anchor=\"middle\" font-size=\"22\" font-family=\"Segoe UI\" fill=\"#ffbf8a\">No data available</text>";
  }

  svg << "<text x=\"" << left << "\" y=\"" << height - 42
      << "\" font-size=\"14\" font-family=\"Segoe UI\" fill=\"#90aac6\">min "
      << FormatNumber(bounds.minX, 3) << "</text>";
  svg << "<text x=\"" << left + plotWidth << "\" y=\"" << height - 42
      << "\" text-anchor=\"end\" font-size=\"14\" font-family=\"Segoe UI\" fill=\"#90aac6\">max "
      << FormatNumber(bounds.maxX, 3) << "</text>";
  svg << "<text x=\"54\" y=\"" << top + 14
      << "\" text-anchor=\"end\" font-size=\"14\" font-family=\"Segoe UI\" fill=\"#90aac6\">"
      << FormatNumber(bounds.maxY, 3) << "</text>";
  svg << "<text x=\"54\" y=\"" << top + plotHeight
      << "\" text-anchor=\"end\" font-size=\"14\" font-family=\"Segoe UI\" fill=\"#90aac6\">"
      << FormatNumber(bounds.minY, 3) << "</text>";
  svg << "</svg>";
  return svg.str();
}

static std::string BuildTrajectoryCsv(const std::vector<ballistics::PlotPoint>& points) {
  std::ostringstream out;
  out << "time_s,longitude_deg,latitude_deg,height_m,speed_mps,theta_deg,psi_deg\n";
  for (const auto& point : points) {
    out << FormatNumber(point.t, 6) << "," << FormatNumber(point.longitudeDeg, 6) << ","
        << FormatNumber(point.latitudeDeg, 6) << "," << FormatNumber(point.height, 3)
        << "," << FormatNumber(point.speed, 3) << "," << FormatNumber(point.thetaDeg, 4)
        << "," << FormatNumber(point.psiDeg, 4) << "\n";
  }
  return out.str();
}

static std::string BuildAttackBoundaryCsv(
    const std::vector<ballistics::BoundaryPoint>& boundary) {
  std::ostringstream out;
  out << "heading_deg,theta_deg,range_m,longitude_deg,latitude_deg,top_height_m\n";
  for (const auto& point : boundary) {
    out << FormatNumber(point.headingDeg, 4) << "," << FormatNumber(point.thetaDeg, 4)
        << "," << FormatNumber(point.range, 2) << "," << FormatNumber(point.longitudeDeg, 6)
        << "," << FormatNumber(point.latitudeDeg, 6) << ","
        << FormatNumber(point.topHeight, 2) << "\n";
  }
  return out.str();
}

static std::string BuildComparisonCsv(
    const std::vector<ballistics::ComparisonRecord>& comparisons) {
  std::ostringstream out;
  out << "method,theta_deg,psi_deg,range_m,flight_time_s,steps,peak_altitude_m,max_speed_mps,"
         "impact_longitude_deg,impact_latitude_deg\n";
  for (const auto& row : comparisons) {
    out << EscapeCsvField(row.name) << "," << FormatNumber(row.thetaDeg, 4) << ","
        << FormatNumber(row.psiDeg, 4) << ","
        << FormatNumber(row.range, 2) << "," << FormatNumber(row.flightTime, 3) << ","
        << row.steps << "," << FormatNumber(row.peakAltitude, 3) << ","
        << FormatNumber(row.maxSpeed, 3) << ","
        << FormatNumber(row.impactLongitudeDeg, 6) << ","
        << FormatNumber(row.impactLatitudeDeg, 6) << "\n";
  }
  return out.str();
}

static std::string BuildSingleComparisonSummaryCsv(
    const ballistics::ComparisonRecord& comparison) {
  std::ostringstream out;
  out << "method,theta_deg,psi_deg,range_m,flight_time_s,steps,peak_altitude_m,max_speed_mps,"
         "impact_longitude_deg,impact_latitude_deg\n";
  out << EscapeCsvField(comparison.name) << ","
      << FormatNumber(comparison.thetaDeg, 4) << ","
      << FormatNumber(comparison.psiDeg, 4) << ","
      << FormatNumber(comparison.range, 2) << ","
      << FormatNumber(comparison.flightTime, 3) << ","
      << comparison.steps << ","
      << FormatNumber(comparison.peakAltitude, 3) << ","
      << FormatNumber(comparison.maxSpeed, 3) << ","
      << FormatNumber(comparison.impactLongitudeDeg, 6) << ","
      << FormatNumber(comparison.impactLatitudeDeg, 6) << "\n";
  return out.str();
}

static std::string BuildSpeedCurveCsv(const std::vector<ballistics::PlotPoint>& points) {
  std::ostringstream out;
  out << "time_s,speed_mps,theta_deg,psi_deg,height_m\n";
  for (const auto& point : points) {
    out << FormatNumber(point.t, 6) << "," << FormatNumber(point.speed, 3) << ","
        << FormatNumber(point.thetaDeg, 4) << "," << FormatNumber(point.psiDeg, 4)
        << "," << FormatNumber(point.height, 3) << "\n";
  }
  return out.str();
}

static const char* AlphaScheduleName(ballistics::AlphaScheduleMode mode);
static const char* AlphaScheduleHelp(ballistics::AlphaScheduleMode mode);
static std::string EnabledMethodsText(const ballistics::SolverConfig& config);

static std::string BuildTrajectoryProfileCsv(const ballistics::SolverConfig& config,
                                             const std::vector<ballistics::PlotPoint>& points) {
  std::ostringstream out;
  out << "time_s,ground_range_km,height_km,speed_mps,theta_deg,psi_deg\n";
  const double launchLambda = DegToRad(config.vehicle.initialLongitudeDeg);
  const double launchPhi = DegToRad(config.vehicle.initialLatitudeDeg);
  for (const auto& point : points) {
    const double groundRangeKm =
        ballistics::greatCircleDistance(launchLambda, launchPhi, DegToRad(point.longitudeDeg),
                                        DegToRad(point.latitudeDeg)) /
        1000.0;
    out << FormatNumber(point.t, 6) << "," << FormatNumber(groundRangeKm, 6) << ","
        << FormatNumber(point.height / 1000.0, 6) << ","
        << FormatNumber(point.speed, 3) << "," << FormatNumber(point.thetaDeg, 4) << ","
        << FormatNumber(point.psiDeg, 4) << "\n";
  }
  return out.str();
}

static std::string BuildAnalysisSummaryCsv(const ballistics::SolverConfig& config,
                                           const ballistics::FinalResult& result) {
  const auto impactPoint = result.optimalHermitePoints.empty()
                               ? ballistics::PlotPoint{}
                               : result.optimalHermitePoints.back();
  double minRange = 0.0;
  double maxRange = 0.0;
  if (!result.attackBoundary.empty()) {
    minRange = result.attackBoundary.front().range;
    maxRange = result.attackBoundary.front().range;
    for (const auto& point : result.attackBoundary) {
      minRange = std::min(minRange, point.range);
      maxRange = std::max(maxRange, point.range);
    }
  }

  std::ostringstream out;
  out << "alpha_schedule,peak_alpha_deg,launch_longitude_deg,launch_latitude_deg,launch_height_m,initial_speed_mps,"
         "initial_theta_deg,initial_psi_deg,optimal_theta_deg,optimal_psi_deg,optimal_range_m,"
         "flight_time_s,peak_altitude_m,max_speed_mps,impact_longitude_deg,impact_latitude_deg,"
         "attack_heading_count,attack_range_min_m,attack_range_max_m,attack_range_spread_m,"
         "method_count,target_enabled,target_reachable,target_miss_distance_m\n";
  out << EscapeCsvField(AlphaScheduleName(config.vehicle.alphaScheduleMode)) << ","
      << FormatNumber(config.vehicle.initialAlphaDeg, 4) << ","
      << FormatNumber(config.vehicle.initialLongitudeDeg, 6) << ","
      << FormatNumber(config.vehicle.initialLatitudeDeg, 6) << ","
      << FormatNumber(config.vehicle.initialHeight, 3) << ","
      << FormatNumber(config.vehicle.initialSpeed, 3) << ","
      << FormatNumber(config.vehicle.initialThetaDeg, 4) << ","
      << FormatNumber(config.vehicle.initialPsiDeg, 4) << ","
      << FormatNumber(result.optimalThetaDeg, 4) << ","
      << FormatNumber(result.optimalPsiDeg, 4) << ","
      << FormatNumber(result.optimalRange, 2) << ","
      << FormatNumber(result.optimalTrajectory.impactTime, 4) << ","
      << FormatNumber(result.optimalTrajectory.maxAltitude, 3) << ","
      << FormatNumber(result.optimalTrajectory.maxSpeed, 3) << ","
      << FormatNumber(impactPoint.longitudeDeg, 6) << ","
      << FormatNumber(impactPoint.latitudeDeg, 6) << ","
      << result.attackBoundary.size() << ","
      << FormatNumber(minRange, 2) << ","
      << FormatNumber(maxRange, 2) << ","
      << FormatNumber(maxRange - minRange, 2) << ","
      << result.comparisons.size() << ","
      << (result.targetSolution.enabled ? "1" : "0") << ","
      << (result.targetSolution.reachable ? "1" : "0") << ","
      << FormatNumber(result.targetSolution.missDistance, 3) << "\n";
  return out.str();
}

static std::string BuildRunParametersCsv(const ballistics::SolverConfig& config) {
  std::ostringstream out;
  out << "parameter,value\n";
  out << "solve_mode," << EscapeCsvField(config.target.enabled ? "Target point" : "Farthest range")
      << "\n";
  out << "alpha_schedule," << EscapeCsvField(AlphaScheduleName(config.vehicle.alphaScheduleMode))
      << "\n";
  out << "peak_alpha_deg," << FormatNumber(config.vehicle.initialAlphaDeg, 4) << "\n";
  out << "initial_beta_deg," << FormatNumber(config.vehicle.initialBetaDeg, 4) << "\n";
  out << "initial_speed_mps," << FormatNumber(config.vehicle.initialSpeed, 3) << "\n";
  out << "initial_theta_deg," << FormatNumber(config.vehicle.initialThetaDeg, 4) << "\n";
  out << "initial_psi_deg," << FormatNumber(config.vehicle.initialPsiDeg, 4) << "\n";
  out << "launch_longitude_deg," << FormatNumber(config.vehicle.initialLongitudeDeg, 6) << "\n";
  out << "launch_latitude_deg," << FormatNumber(config.vehicle.initialLatitudeDeg, 6) << "\n";
  out << "launch_height_m," << FormatNumber(config.vehicle.initialHeight, 3) << "\n";
  out << "mass_kg," << FormatNumber(config.vehicle.mass, 3) << "\n";
  out << "reference_area_m2," << FormatNumber(config.vehicle.refArea, 6) << "\n";
  out << "earth_radius_m," << FormatNumber(config.earth.radius, 3) << "\n";
  out << "earth_mu_m3ps2," << FormatNumber(config.earth.mu, 3) << "\n";
  out << "earth_omega_radps," << FormatNumber(config.earth.omega, 9) << "\n";
  out << "enabled_methods," << EscapeCsvField(EnabledMethodsText(config)) << "\n";
  out << "fixed_step_s," << FormatNumber(config.integration.fixedStep, 4) << "\n";
  out << "adaptive_step_s," << FormatNumber(config.integration.adaptiveStep, 4) << "\n";
  out << "adaptive_min_step_s," << FormatNumber(config.integration.adaptiveMinStep, 4) << "\n";
  out << "adaptive_max_step_s," << FormatNumber(config.integration.adaptiveMaxStep, 4) << "\n";
  out << "adaptive_tolerance," << FormatNumber(config.integration.adaptiveTolerance, 8) << "\n";
  out << "search_min_theta_deg," << FormatNumber(config.search.minThetaDeg, 4) << "\n";
  out << "search_max_theta_deg," << FormatNumber(config.search.maxThetaDeg, 4) << "\n";
  out << "search_theta_tolerance_deg," << FormatNumber(config.search.thetaToleranceDeg, 8)
      << "\n";
  out << "attack_heading_count," << config.search.attackZoneHeadingCount << "\n";
  out << "attack_theta_min_deg," << FormatNumber(config.search.attackThetaMinDeg, 4) << "\n";
  out << "attack_theta_max_deg," << FormatNumber(config.search.attackThetaMaxDeg, 4) << "\n";
  out << "attack_theta_samples," << config.search.attackThetaSamples << "\n";
  out << "ga_population," << config.search.geneticPopulation << "\n";
  out << "ga_elite_count," << config.search.geneticEliteCount << "\n";
  out << "ga_generation_count," << config.search.geneticGenerationCount << "\n";
  out << "ga_random_seed," << config.search.geneticRandomSeed << "\n";
  out << "ga_mutation_rate," << FormatNumber(config.search.geneticMutationRate, 6) << "\n";
  out << "ga_mutation_sigma_deg," << FormatNumber(config.search.geneticMutationSigmaDeg, 4)
      << "\n";
  out << "target_enabled," << (config.target.enabled ? "1" : "0") << "\n";
  out << "target_longitude_deg," << FormatNumber(config.target.longitudeDeg, 6) << "\n";
  out << "target_latitude_deg," << FormatNumber(config.target.latitudeDeg, 6) << "\n";
  out << "target_height_m," << FormatNumber(config.target.heightM, 3) << "\n";
  out << "target_tolerance_m," << FormatNumber(config.target.toleranceMeters, 3) << "\n";
  return out.str();
}

static std::string BuildImpactPointCsv(const ballistics::FinalResult& result) {
  const auto impactPoint = result.optimalHermitePoints.empty()
                               ? ballistics::PlotPoint{}
                               : result.optimalHermitePoints.back();
  std::ostringstream out;
  out << "label,longitude_deg,latitude_deg,height_m,range_m,flight_time_s,theta_deg,psi_deg\n";
  out << "farthest_impact," << FormatNumber(impactPoint.longitudeDeg, 6) << ","
      << FormatNumber(impactPoint.latitudeDeg, 6) << ","
      << FormatNumber(impactPoint.height, 3) << ","
      << FormatNumber(result.optimalRange, 2) << ","
      << FormatNumber(result.optimalTrajectory.impactTime, 4) << ","
      << FormatNumber(result.optimalThetaDeg, 4) << ","
      << FormatNumber(result.optimalPsiDeg, 4) << "\n";
  return out.str();
}

static std::string BuildAttackTrajectoryIndexCsv(const ballistics::FinalResult& result) {
  std::ostringstream out;
  out << "sample_id,heading_deg,point_count,end_longitude_deg,end_latitude_deg\n";
  for (std::size_t i = 0; i < result.attackTrajectories.size(); ++i) {
    const auto& trajectory = result.attackTrajectories[i];
    const auto& impact = trajectory.empty() ? ballistics::PlotPoint{} : trajectory.back();
    const double headingDeg = i < result.attackTrajectoryHeadings.size()
                                  ? result.attackTrajectoryHeadings[i]
                                  : 0.0;
    out << (i + 1) << "," << FormatNumber(headingDeg, 4) << ","
        << trajectory.size() << "," << FormatNumber(impact.longitudeDeg, 6) << ","
        << FormatNumber(impact.latitudeDeg, 6) << "\n";
  }
  return out.str();
}

static std::string EnabledMethodsText(const ballistics::SolverConfig& config) {
  std::vector<std::string> names;
  if (config.methods.fixedLinear) {
    names.push_back("Fixed RK4 + linear");
  }
  if (config.methods.fixedHermite) {
    names.push_back("Fixed RK4 + Hermite");
  }
  if (config.methods.adaptiveHermite) {
    names.push_back("Adaptive RK4 + Hermite");
  }
  if (config.methods.geneticAdaptive) {
    names.push_back("Genetic + adaptive");
  }
  if (names.empty()) {
    return "None";
  }
  std::ostringstream out;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << names[i];
  }
  return out.str();
}

static const char* AlphaScheduleName(ballistics::AlphaScheduleMode mode) {
  switch (mode) {
    case ballistics::AlphaScheduleMode::ConstantBelow30km:
      return "Baseline constant";
    case ballistics::AlphaScheduleMode::SmoothGlide:
      return "Smooth glide";
    case ballistics::AlphaScheduleMode::QianStyle:
      return "Qian-style";
    case ballistics::AlphaScheduleMode::AggressiveSkip:
      return "Aggressive skip";
    case ballistics::AlphaScheduleMode::QianSkipGlide:
      return "Qian skip-glide";
    case ballistics::AlphaScheduleMode::SangerStyle:
      return "Sanger-style";
  }
  return "Unknown";
}

static const char* AlphaScheduleHelp(ballistics::AlphaScheduleMode mode) {
  switch (mode) {
    case ballistics::AlphaScheduleMode::ConstantBelow30km:
      return "30 km above keeps alpha at 0 deg; below 30 km it jumps directly to the configured peak alpha.";
    case ballistics::AlphaScheduleMode::SmoothGlide:
      return "High altitude stays clean, mid-altitude raises alpha smoothly, and low altitude eases the angle back for a stable glide.";
    case ballistics::AlphaScheduleMode::QianStyle:
      return "A stronger mid-altitude lift build-up that creates a more dramatic glide-turn profile without being too numerically wild.";
    case ballistics::AlphaScheduleMode::AggressiveSkip:
      return "A high-energy schedule with a stronger mid-course alpha plateau for the boldest long-range glide shape.";
    case ballistics::AlphaScheduleMode::QianSkipGlide:
      return "A skip-glide flavored schedule: strong upper-atmosphere pull-up, low-alpha unloading after the rebound, then a second lift build-up in denser air.";
    case ballistics::AlphaScheduleMode::SangerStyle:
      return "A long-range boost-glide flavored schedule with a sustained upper-atmosphere glide plateau before the terminal descent.";
  }
  return "";
}

static std::string BuildComputedDataMarkdown(const ballistics::SolverConfig& config,
                                             const ballistics::FinalResult& result) {
  const auto& target = result.targetSolution;
  std::ostringstream out;
  out << "# Computed Data\n\n";
  out << "## Launch State\n";
  out << "- Launch longitude: " << FormatNumber(config.vehicle.initialLongitudeDeg, 4)
      << " deg\n";
  out << "- Launch latitude: " << FormatNumber(config.vehicle.initialLatitudeDeg, 4)
      << " deg\n";
  out << "- Launch height: " << FormatNumber(config.vehicle.initialHeight / 1000.0, 3)
      << " km\n";
  out << "- Initial speed: " << FormatNumber(config.vehicle.initialSpeed, 2) << " m/s\n";
  out << "- Initial theta: " << FormatNumber(config.vehicle.initialThetaDeg, 3) << " deg\n";
  out << "- Initial psi: " << FormatNumber(config.vehicle.initialPsiDeg, 3) << " deg\n";
  out << "- Alpha schedule: " << AlphaScheduleName(config.vehicle.alphaScheduleMode) << "\n";
  out << "- Peak alpha: " << FormatNumber(config.vehicle.initialAlphaDeg, 3) << " deg\n";
  out << "- Enabled methods: " << EnabledMethodsText(config) << "\n\n";
  out << "## Optimal Trajectory\n";
  out << "- Optimal theta: " << FormatNumber(result.optimalThetaDeg, 4) << " deg\n";
  out << "- Optimal heading: " << FormatNumber(result.optimalPsiDeg, 4) << " deg\n";
  out << "- Range: " << FormatNumber(result.optimalRange / 1000.0, 3) << " km\n";
  out << "- Flight time: " << FormatNumber(result.optimalTrajectory.impactTime, 3) << " s\n";
  out << "- Peak altitude: "
      << FormatNumber(result.optimalTrajectory.maxAltitude / 1000.0, 3) << " km\n";
  if (!result.optimalHermitePoints.empty()) {
    const auto& impactPoint = result.optimalHermitePoints.back();
    out << "- Impact longitude: " << FormatNumber(impactPoint.longitudeDeg, 4) << " deg\n";
    out << "- Impact latitude: " << FormatNumber(impactPoint.latitudeDeg, 4) << " deg\n";
  }
  out << "- Accepted integration steps: " << result.optimalTrajectory.acceptedSteps << "\n";
  out << "- Max speed: " << FormatNumber(result.optimalTrajectory.maxSpeed, 3) << " m/s\n";
  out << "- Valid attack-zone headings: " << result.attackBoundary.size() << "\n\n";
  if (!result.attackBoundary.empty()) {
    double minRange = result.attackBoundary.front().range;
    double maxRange = result.attackBoundary.front().range;
    for (const auto& point : result.attackBoundary) {
      minRange = std::min(minRange, point.range);
      maxRange = std::max(maxRange, point.range);
    }
    out << "## Attack-Zone Statistics\n";
    out << "- Minimum boundary range: " << FormatNumber(minRange / 1000.0, 3) << " km\n";
    out << "- Maximum boundary range: " << FormatNumber(maxRange / 1000.0, 3) << " km\n";
    out << "- Boundary spread: " << FormatNumber((maxRange - minRange) / 1000.0, 3)
        << " km\n\n";
  }
  out << "## Target Data\n";
  if (target.enabled) {
    out << "- Reachable: " << (target.reachable ? "Yes" : "No") << "\n";
    out << "- Target point: " << FormatNumber(target.targetLongitudeDeg, 4) << " deg, "
        << FormatNumber(target.targetLatitudeDeg, 4) << " deg, "
        << FormatNumber(target.targetHeightM / 1000.0, 3) << " km\n";
    out << "- Miss distance: " << FormatNumber(target.missDistance / 1000.0, 3) << " km\n";
    out << "- Target theta: " << FormatNumber(target.thetaDeg, 4) << " deg\n";
    out << "- Target psi: " << FormatNumber(target.psiDeg, 4) << " deg\n";
  } else {
    out << "- Target solver disabled for this run.\n";
  }
  return out.str();
}

static std::string BuildTrajectorySvg(const ballistics::SolverConfig& config,
                                      const std::vector<ballistics::PlotPoint>& points) {
  std::vector<SvgPoint> profile;
  profile.reserve(points.size());
  const double launchLambda = DegToRad(config.vehicle.initialLongitudeDeg);
  const double launchPhi = DegToRad(config.vehicle.initialLatitudeDeg);
  for (const auto& point : points) {
    const double groundRangeKm =
        ballistics::greatCircleDistance(launchLambda, launchPhi, DegToRad(point.longitudeDeg),
                                        DegToRad(point.latitudeDeg)) /
        1000.0;
    profile.push_back({groundRangeKm, point.height / 1000.0});
  }
  return BuildPolylineSvg("Optimal Trajectory Profile", "Ground range (km)",
                          "Height (km)", profile, "#ffcf5a");
}

static std::string BuildSpeedTimeSvg(const std::vector<ballistics::PlotPoint>& points) {
  std::vector<SvgPoint> speedCurve;
  speedCurve.reserve(points.size());
  for (const auto& point : points) {
    speedCurve.push_back({point.t, point.speed});
  }
  return BuildPolylineSvg("Speed-Time Curve", "Time (s)", "Speed (m/s)",
                          speedCurve, "#7ad3ff");
}

static std::string BuildAttackZoneSvg(const ballistics::SolverConfig& config,
                                      const ballistics::FinalResult& result) {
  std::vector<SvgPoint> polygon;
  polygon.reserve(result.attackBoundary.size());
  for (const auto& point : result.attackBoundary) {
    polygon.push_back({point.longitudeDeg, point.latitudeDeg});
  }
  if (polygon.empty()) {
    return BuildPolylineSvg("Attack Zone Projection", "Longitude (deg)",
                            "Latitude (deg)", polygon, "#3fe7ff", "#3fe7ff", true);
  }
  std::string svg = BuildPolylineSvg("Attack Zone Projection", "Longitude (deg)",
                                     "Latitude (deg)", polygon, "#3fe7ff",
                                     "#3fe7ff", true);
  const SvgBounds bounds = ComputeBounds(polygon);
  constexpr double width = 960.0;
  constexpr double height = 560.0;
  constexpr double left = 90.0;
  constexpr double right = 28.0;
  constexpr double top = 56.0;
  constexpr double bottom = 72.0;
  const double plotWidth = width - left - right;
  const double plotHeight = height - top - bottom;
  const auto mapX = [&](double value) {
    return left + (value - bounds.minX) / (bounds.maxX - bounds.minX) * plotWidth;
  };
  const auto mapY = [&](double value) {
    return top + plotHeight - (value - bounds.minY) / (bounds.maxY - bounds.minY) * plotHeight;
  };
  const double launchX = mapX(config.vehicle.initialLongitudeDeg);
  const double launchY = mapY(config.vehicle.initialLatitudeDeg);
  const auto impactPoint = result.optimalHermitePoints.empty()
                               ? ballistics::PlotPoint{}
                               : result.optimalHermitePoints.back();
  const double impactX = mapX(impactPoint.longitudeDeg);
  const double impactY = mapY(impactPoint.latitudeDeg);
  const std::string closingTag = "</svg>";
  const std::size_t closingPos = svg.rfind(closingTag);
  if (closingPos != std::string::npos) {
    std::ostringstream overlay;
    overlay << "<circle cx=\"" << FormatNumber(launchX, 2) << "\" cy=\""
            << FormatNumber(launchY, 2)
            << "\" r=\"7\" fill=\"#7cffaf\" stroke=\"#d9ffe7\" stroke-width=\"2\"/>";
    overlay << "<text x=\"" << FormatNumber(launchX + 12.0, 2) << "\" y=\""
            << FormatNumber(launchY - 10.0, 2)
            << "\" font-size=\"16\" font-family=\"Segoe UI\" fill=\"#dff7e8\">Launch</text>";
    if (!result.optimalHermitePoints.empty()) {
      overlay << "<circle cx=\"" << FormatNumber(impactX, 2) << "\" cy=\""
              << FormatNumber(impactY, 2)
              << "\" r=\"6\" fill=\"#ff6a6a\" stroke=\"#ffe0e0\" stroke-width=\"2\"/>";
      overlay << "<text x=\"" << FormatNumber(impactX + 12.0, 2) << "\" y=\""
              << FormatNumber(impactY - 10.0, 2)
              << "\" font-size=\"16\" font-family=\"Segoe UI\" fill=\"#ffe7e7\">Impact</text>";
    }
    svg.insert(closingPos, overlay.str());
  }
  return svg;
}

class BallisticApp final : public oe::ImGuiEventHandler {
 public:
  BallisticApp(osgViewer::Viewer* viewer, oe::MapNode* mapNode,
               oeu::EarthManipulator* manip, osg::MatrixTransform* worldSpin)
      : viewer_(viewer),
        mapNode_(mapNode),
        manip_(manip),
        worldSpin_(worldSpin),
        geoSrs_(mapNode->getMapSRS()->getGeographicSRS()),
        config_(ballistics::makeDefaultConfig()),
        worldSpinStart_(std::chrono::steady_clock::now()),
        trajectoryAnimationStart_(worldSpinStart_) {
    config_.output.livePath.clear();
    config_.output.finalPath.clear();
    config_.output.liveDelayMs = 0.0;

    overlay_ = new osg::Group();
    static_ = new osg::Group();
    dynamic_ = new osg::Group();
    overlay_->addChild(static_);
    overlay_->addChild(dynamic_);
    buildGraticules();

    state_.status = ballistics::SolverStatus::Idle;
    state_.message = "Edit parameters and click Compute.";
  }

  ~BallisticApp() override {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  osg::Group* overlayRoot() const { return overlay_.get(); }

  void draw(osg::RenderInfo&) override {
    if (!busy_.load() && worker_.joinable()) {
      worker_.join();
    }

    updateWorldRotation();

    ballistics::LiveState stateCopy;
    std::uint64_t version = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stateCopy = state_;
      version = version_;
    }
    const bool done = stateCopy.status == ballistics::SolverStatus::Done &&
                      stateCopy.hasFinalResult;
    if (done && version != trajectoryAnimationVersion_) {
      trajectoryAnimationVersion_ = version;
      trajectoryAnimationStart_ = std::chrono::steady_clock::now();
      focusedResultVersion_ = std::numeric_limits<std::uint64_t>::max();
    } else if (!done) {
      trajectoryAnimationVersion_ = std::numeric_limits<std::uint64_t>::max();
      focusedResultVersion_ = std::numeric_limits<std::uint64_t>::max();
    }

    const double trajectoryAnimationProgress = currentTrajectoryAnimationProgress(stateCopy);
    const bool animationChanged =
        std::abs(trajectoryAnimationProgress - renderedAnimationProgress_) > 1e-3;
    const bool animating = done && trajectoryAnimationProgress < 0.999;

    if (version != renderedVersion_ || (done && animationChanged)) {
      rebuildScene(stateCopy, trajectoryAnimationProgress);
      renderedVersion_ = version;
      renderedAnimationProgress_ = trajectoryAnimationProgress;
      if (animating) {
        viewer_->requestRedraw();
      }
    }
    drawPanel(stateCopy);
  }

 private:
  void updateWorldRotation() const {
    if (worldSpin_ == nullptr) {
      return;
    }
    constexpr double kVisualRotationScale = 240.0;
    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = now - worldSpinStart_;
    const double angleRad =
        config_.earth.omega * elapsed.count() * kVisualRotationScale;
    worldSpin_->setMatrix(osg::Matrixd::rotate(angleRad, osg::Vec3d(0.0, 0.0, 1.0)));
  }

  oe::GeoPoint geo(double lonDeg, double latDeg, double heightM) const {
    return {geoSrs_.get(), lonDeg, latDeg, heightM, oe::ALTMODE_ABSOLUTE};
  }

  osg::Vec3d world(double lonDeg, double latDeg, double heightM) const {
    osg::Vec3d out;
    geo(lonDeg, latDeg, heightM).toWorld(out);
    return out;
  }

  void finishOverlayNode(osg::Node* node) const {
    if (node == nullptr) {
      return;
    }
    osg::StateSet* ss = node->getOrCreateStateSet();
    oe::GLUtils::setGlobalDefaults(ss);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    oe::Registry::shaderGenerator().run(node);
  }

  oe::Style labelStyle(short dx, short dy) const {
    oe::Style style;
    auto* text = style.getOrCreate<oe::TextSymbol>();
    text->fill() = oe::Fill(oe::Color(0.95f, 0.98f, 1.0f, 1.0f));
    text->halo() = oe::Stroke(oe::Color(0.05f, 0.07f, 0.12f, 0.95f));
    text->size() = 15.0f;
    text->pixelOffset() = osg::Vec2s(dx, dy);
    text->alignment() = oe::TextSymbol::ALIGN_LEFT_TOP;
    text->declutter() = false;
    return style;
  }

  osg::ref_ptr<oe::PlaceNode> label(double lonDeg, double latDeg, double heightM,
                                    const std::string& text, short dx,
                                    short dy) const {
    return new oe::PlaceNode(geo(lonDeg, latDeg, heightM), text,
                             labelStyle(dx, dy));
  }

  osg::ref_ptr<osg::Geode> sphere(double lonDeg, double latDeg, double heightM,
                                  float radiusM,
                                  const osg::Vec4f& color) const {
    auto geode = new osg::Geode();
    auto drawable =
        new osg::ShapeDrawable(new osg::Sphere(world(lonDeg, latDeg, heightM), radiusM));
    drawable->setColor(color);
    drawable->setUseVertexBufferObjects(true);
    geode->addDrawable(drawable);
    auto* ss = geode->getOrCreateStateSet();
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    auto* material = new osg::Material();
    material->setColorMode(osg::Material::OFF);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    material->setAmbient(
        osg::Material::FRONT_AND_BACK,
        osg::Vec4f(color.r() * 0.45f, color.g() * 0.45f, color.b() * 0.45f, color.a()));
    material->setEmission(
        osg::Material::FRONT_AND_BACK,
        osg::Vec4f(color.r() * 0.18f, color.g() * 0.18f, color.b() * 0.18f, color.a()));
    ss->setAttributeAndModes(material, osg::StateAttribute::ON);
    finishOverlayNode(geode);
    return geode;
  }

  osg::ref_ptr<osg::Geode> line(const std::vector<osg::Vec3d>& pts,
                                const osg::Vec4f& color, float width,
                                bool depthTest = true) const {
    if (pts.size() < 2) {
      return {};
    }
    auto verts = new osg::Vec3dArray();
    for (const auto& p : pts) {
      verts->push_back(p);
    }
    auto colors = new osg::Vec4Array();
    colors->push_back(color);
    auto geom = new osg::Geometry();
    geom->setUseDisplayList(false);
    geom->setUseVertexBufferObjects(true);
    geom->setVertexArray(verts);
    geom->setColorArray(colors, osg::Array::BIND_OVERALL);
    geom->addPrimitiveSet(
        new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(verts->size())));
    auto geode = new osg::Geode();
    geode->addDrawable(geom);
    auto* ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(new osg::LineWidth(width), osg::StateAttribute::ON);
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    ss->setMode(GL_DEPTH_TEST, depthTest ? osg::StateAttribute::ON
                                         : (osg::StateAttribute::OFF |
                                            osg::StateAttribute::OVERRIDE));
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    if (!depthTest) {
      ss->setAttributeAndModes(new osg::Depth(osg::Depth::ALWAYS, 0.0, 1.0, false),
                               osg::StateAttribute::ON);
      ss->setRenderBinDetails(60, "RenderBin");
    }
    finishOverlayNode(geode);
    return geode;
  }

  std::vector<osg::Vec3d> path(const std::vector<ballistics::PlotPoint>& points,
                               double bias = 0.0) const {
    std::vector<osg::Vec3d> out;
    out.reserve(points.size());
    for (const auto& p : points) {
      out.push_back(world(p.longitudeDeg, p.latitudeDeg, std::max(0.0, p.height + bias)));
    }
    return out;
  }

  std::vector<osg::Vec3d> surfaceProjectionPath(
      const std::vector<ballistics::PlotPoint>& points, double surfaceHeightM = 1'200.0) const {
    std::vector<osg::Vec3d> out;
    out.reserve(points.size());
    for (const auto& p : points) {
      out.push_back(world(p.longitudeDeg, p.latitudeDeg, surfaceHeightM));
    }
    return out;
  }

  std::vector<ballistics::PlotPoint> boundaryLoop(
      const std::vector<ballistics::BoundaryPoint>& boundary, double heightScale = 1.0,
      double minimumHeightM = 25'000.0) const {
    std::vector<ballistics::PlotPoint> loop;
    if (boundary.size() < 3) {
      return loop;
    }

    loop.reserve(boundary.size() + 1);
    for (const auto& point : boundary) {
      loop.push_back({0.0, point.longitudeDeg, point.latitudeDeg,
                      std::max(minimumHeightM, point.topHeight * heightScale), 0.0,
                      0.0, 0.0});
    }

    loop.push_back(loop.front());
    return loop;
  }

  std::vector<ballistics::PlotPoint> scaledBoundaryLoop(
      const std::vector<ballistics::BoundaryPoint>& boundary, std::size_t segmentCount = 4,
      double radiusScale = 1.0, double heightScale = 1.0,
      double minimumHeightM = 25'000.0) const {
    std::vector<ballistics::PlotPoint> loop;
    if (boundary.size() < 3) {
      return loop;
    }

    const double lon0 = config_.vehicle.initialLongitudeDeg;
    const double lat0 = config_.vehicle.initialLatitudeDeg;
    const double lambda0 = DegToRad(lon0);
    const double phi0 = DegToRad(lat0);

    const std::size_t subdivisions = std::max<std::size_t>(1, segmentCount);
    loop.reserve(boundary.size() * subdivisions + 1);
    for (std::size_t i = 0; i < boundary.size(); ++i) {
      const auto& a = boundary[i];
      const auto& b = boundary[(i + 1) % boundary.size()];
      double headingB = b.headingDeg;
      if (headingB < a.headingDeg) {
        headingB += 360.0;
      }

      for (std::size_t s = 0; s < subdivisions; ++s) {
        const double t =
            static_cast<double>(s) / static_cast<double>(subdivisions);
        const double headingDeg = a.headingDeg + (headingB - a.headingDeg) * t;
        const double bearing = DegToRad(headingDeg);
        const double radiusM =
            std::max(0.0, (a.range + (b.range - a.range) * t) * radiusScale);
        const double displayHeightM = std::max(
            minimumHeightM,
            (a.topHeight + (b.topHeight - a.topHeight) * t) * heightScale);
        const double angularDistance = radiusM / config_.earth.radius;

        const double sinPhi2 =
            std::sin(phi0) * std::cos(angularDistance) +
            std::cos(phi0) * std::sin(angularDistance) * std::cos(bearing);
        const double phi2 = std::asin(std::clamp(sinPhi2, -1.0, 1.0));
        const double lambda2 =
            lambda0 +
            std::atan2(std::sin(bearing) * std::sin(angularDistance) * std::cos(phi0),
                       std::cos(angularDistance) - std::sin(phi0) * std::sin(phi2));
        loop.push_back(
            {0.0, RadToDeg(lambda2), RadToDeg(phi2), displayHeightM, 0.0, 0.0, 0.0});
      }
    }

    if (!loop.empty()) {
      loop.push_back(loop.front());
    }
    return loop;
  }

  static ballistics::PlotPoint interpolatePoint(const ballistics::PlotPoint& a,
                                                const ballistics::PlotPoint& b,
                                                double t) {
    const auto lerp = [t](double x, double y) { return x + (y - x) * t; };
    return {lerp(a.t, b.t),
            lerp(a.longitudeDeg, b.longitudeDeg),
            lerp(a.latitudeDeg, b.latitudeDeg),
            lerp(a.height, b.height),
            lerp(a.speed, b.speed),
            lerp(a.thetaDeg, b.thetaDeg),
            lerp(a.psiDeg, b.psiDeg)};
  }

  std::vector<ballistics::PlotPoint> revealedTrajectory(
      const std::vector<ballistics::PlotPoint>& points, double progress) const {
    if (points.size() < 2 || progress >= 0.999) {
      return points;
    }
    if (progress <= 0.0) {
      return {points.front()};
    }

    const double endTime = std::max(points.back().t, 1e-6);
    const double revealTime = endTime * progress;
    std::vector<ballistics::PlotPoint> out;
    out.reserve(points.size());
    out.push_back(points.front());

    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto& prev = points[i - 1];
      const auto& next = points[i];
      if (next.t <= revealTime) {
        out.push_back(next);
        continue;
      }
      const double span = std::max(next.t - prev.t, 1e-6);
      const double localT = std::clamp((revealTime - prev.t) / span, 0.0, 1.0);
      out.push_back(interpolatePoint(prev, next, localT));
      break;
    }

    return out;
  }

  double currentTrajectoryAnimationProgress(const ballistics::LiveState& state) const {
    const bool done = state.status == ballistics::SolverStatus::Done && state.hasFinalResult;
    if (!done || state.finalResult.optimalHermitePoints.size() < 2) {
      return 1.0;
    }

    constexpr double kAnimationDurationSeconds = 4.0;
    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = now - trajectoryAnimationStart_;
    return std::clamp(elapsed.count() / kAnimationDurationSeconds, 0.0, 1.0);
  }

  void addAttackZoneMarkers(const std::vector<ballistics::BoundaryPoint>& boundary) {
    for (const auto& point : boundary) {
      dynamic_->addChild(sphere(point.longitudeDeg, point.latitudeDeg, 2'400.0,
                                 7'000.0f, osg::Vec4f(0.35f, 0.96f, 1.0f, 0.72f)));
    }
  }

  std::vector<osg::Vec3d> radialGuidePoints(
      const ballistics::PlotPoint& point, double guideHeight) const {
    return {world(config_.vehicle.initialLongitudeDeg, config_.vehicle.initialLatitudeDeg, 0.0),
            world(point.longitudeDeg, point.latitudeDeg, guideHeight)};
  }

  osg::ref_ptr<osg::Geode> attackZone(
      const std::vector<ballistics::PlotPoint>& circle,
      const osg::Vec4f& fillColor) const {
    if (circle.size() < 3) {
      return {};
    }
    auto roofVerts = new osg::Vec3dArray();
    auto roof = new osg::Geometry();
    roof->setUseDisplayList(false);
    roof->setUseVertexBufferObjects(true);
    for (const auto& p : circle) {
      roofVerts->push_back(world(p.longitudeDeg, p.latitudeDeg, p.height));
    }
    roof->setVertexArray(roofVerts);
    auto roofColors = new osg::Vec4Array();
    roofColors->push_back(fillColor);
    roof->setColorArray(roofColors, osg::Array::BIND_OVERALL);
    roof->addPrimitiveSet(
        new osg::DrawArrays(GL_POLYGON, 0, static_cast<GLsizei>(roofVerts->size())));
    osgUtil::Tessellator tess;
    tess.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
    tess.retessellatePolygons(*roof);

    auto geode = new osg::Geode();
    geode->addDrawable(roof);
    auto* ss = geode->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    finishOverlayNode(geode);
    return geode;
  }

  void buildGraticules() {
    const osg::Vec4f c(0.35f, 0.63f, 1.0f, 0.18f);
    for (int lat = -60; lat <= 60; lat += 30) {
      std::vector<osg::Vec3d> pts;
      for (int lon = -180; lon <= 180; lon += 10) {
        pts.push_back(world(static_cast<double>(lon), static_cast<double>(lat), 500.0));
      }
      static_->addChild(line(pts, c, 1.1f));
    }
    for (int lon = -180; lon <= 180; lon += 30) {
      std::vector<osg::Vec3d> pts;
      for (int lat = -90; lat <= 90; lat += 8) {
        pts.push_back(world(static_cast<double>(lon), static_cast<double>(lat), 500.0));
      }
      static_->addChild(line(pts, c, 1.1f));
    }
  }

  void publish(const ballistics::LiveState& s) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = s;
      ++version_;
    }
    viewer_->requestRedraw();
  }

  void startSolve() {
    if (busy_.exchange(true)) {
      return;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    ballistics::SolverConfig run = config_;
    run.target.enabled = mode_ == SolveMode::TargetPoint;
    run.output.livePath.clear();
    run.output.finalPath.clear();
    run.output.liveDelayMs = animate_ ? config_.output.liveDelayMs : 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hasSolvedConfig_ = false;
      lastExportDirectory_.clear();
      exportStatusMessage_.clear();
      exportFailed_ = false;
    }
    {
      std::string modeName = mode_ == SolveMode::TargetPoint ? "target" : "farthest";
      ballistics::logInfo(
          "osgearth_viewer",
          std::string("Starting solve from UI. mode=") + modeName +
              ", launchHeightM=" + std::to_string(run.vehicle.initialHeight) +
              ", targetLonDeg=" + std::to_string(run.target.longitudeDeg) +
              ", targetLatDeg=" + std::to_string(run.target.latitudeDeg));
    }
    publish({ballistics::SolverStatus::Running, "startup", "Preparing solver", 0.0});
    worker_ = std::thread([this, run]() {
      try {
        ballistics::LiveWriter writer([this](const ballistics::LiveState& s) { publish(s); });
        const auto result = ballistics::runSolver(run, &writer);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          solvedConfig_ = run;
          hasSolvedConfig_ = true;
        }
        writer.writeFinalState(result);
        ballistics::logInfo("osgearth_viewer", "Background solve finished successfully.");
      } catch (const std::exception& ex) {
        ballistics::logException("osgearth_viewer", ex);
        ballistics::LiveState failed;
        failed.status = ballistics::SolverStatus::Failed;
        failed.message = "Solver failed";
        failed.errorMessage = ex.what();
        publish(failed);
      } catch (...) {
        ballistics::logError("osgearth_viewer",
                             "Background solve failed with unknown exception.");
        ballistics::LiveState failed;
        failed.status = ballistics::SolverStatus::Failed;
        failed.message = "Solver failed";
        failed.errorMessage = "Unknown exception";
        publish(failed);
      }
      busy_.store(false);
    });
  }

  static SeriesBundle series(const ballistics::LiveState& state) {
    SeriesBundle s;
    if (state.status == ballistics::SolverStatus::Done && state.hasFinalResult &&
        !state.finalResult.comparisons.empty()) {
      static const std::array<ImU32, 4> kColors = {
          IM_COL32(204, 218, 240, 185), IM_COL32(122, 211, 255, 255),
          IM_COL32(255, 184, 92, 255), IM_COL32(140, 255, 172, 255)};
      static const std::array<float, 4> kThickness = {2.8f, 3.0f, 3.4f, 3.0f};
      s.entries.reserve(state.finalResult.comparisons.size());
      for (std::size_t i = 0; i < state.finalResult.comparisons.size(); ++i) {
        const auto& row = state.finalResult.comparisons[i];
        s.entries.push_back({row.name, row.speedCurve, kColors[i % kColors.size()],
                             kThickness[i % kThickness.size()]});
      }
    } else {
      s.entries.push_back({"Current trajectory", state.currentTrajectory,
                           IM_COL32(255, 214, 92, 255), 2.8f});
      s.entries.push_back({"Best trajectory", state.bestTrajectory,
                           IM_COL32(255, 94, 116, 255), 3.2f});
    }
    return s;
  }

  static void HelpMarker(const char* text) {
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
      ImGui::BeginTooltip();
      ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
      ImGui::TextUnformatted(text);
      ImGui::PopTextWrapPos();
      ImGui::EndTooltip();
    }
  }

  static void KeyValueRow(const char* label, const std::string& value,
                          const ImVec4& valueColor = ImVec4(0.96f, 0.98f, 1.0f, 1.0f)) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(label);
    ImGui::TableSetColumnIndex(1);
    ImGui::TextColored(valueColor, "%s", value.c_str());
  }

  bool hasEnabledMethods() const {
    return config_.methods.fixedLinear || config_.methods.fixedHermite ||
           config_.methods.adaptiveHermite || config_.methods.geneticAdaptive;
  }

  bool exportBundle(const ballistics::LiveState& state, std::filesystem::path& exportDir,
                    std::string& errorMessage) {
    if (!(state.status == ballistics::SolverStatus::Done && state.hasFinalResult)) {
      errorMessage = "No completed result is available to export.";
      return false;
    }

    ballistics::SolverConfig solvedConfig;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      solvedConfig = hasSolvedConfig_ ? solvedConfig_ : config_;
    }

    try {
      exportDir = std::filesystem::current_path() / "data" / "exports" / TimestampTag();
      std::filesystem::create_directories(exportDir);

      const auto& result = state.finalResult;
      const bool ok =
          WriteTextFile(exportDir / "run-parameters.csv",
                        BuildRunParametersCsv(solvedConfig)) &&
          WriteTextFile(exportDir / "computed-data.md",
                        BuildComputedDataMarkdown(solvedConfig, result)) &&
          WriteTextFile(exportDir / "analysis-summary.csv",
                        BuildAnalysisSummaryCsv(solvedConfig, result)) &&
          WriteTextFile(exportDir / "impact-point.csv",
                        BuildImpactPointCsv(result)) &&
          WriteTextFile(exportDir / "optimal-trajectory.csv",
                        BuildTrajectoryCsv(result.optimalHermitePoints)) &&
          WriteTextFile(exportDir / "optimal-trajectory-profile.csv",
                        BuildTrajectoryProfileCsv(solvedConfig, result.optimalHermitePoints)) &&
          WriteTextFile(exportDir / "optimal-trajectory-linear.csv",
                        BuildTrajectoryCsv(result.optimalLinearPoints)) &&
          WriteTextFile(exportDir / "speed-time.csv",
                        BuildSpeedCurveCsv(result.speedCurveHermite)) &&
          WriteTextFile(exportDir / "speed-time-linear.csv",
                        BuildSpeedCurveCsv(result.speedCurveLinear)) &&
          WriteTextFile(exportDir / "attack-boundary.csv",
                        BuildAttackBoundaryCsv(result.attackBoundary)) &&
          WriteTextFile(exportDir / "attack-trajectory-index.csv",
                        BuildAttackTrajectoryIndexCsv(result)) &&
          WriteTextFile(exportDir / "method-comparison.csv",
                        BuildComparisonCsv(result.comparisons));

      if (ok) {
        const std::filesystem::path methodsDir = exportDir / "methods";
        std::filesystem::create_directories(methodsDir);
        for (std::size_t i = 0; i < result.comparisons.size(); ++i) {
          const auto& comparison = result.comparisons[i];
          std::ostringstream folderName;
          folderName << std::setw(2) << std::setfill('0') << (i + 1) << "-"
                     << SlugifyFilename(comparison.name);
          const std::filesystem::path methodDir = methodsDir / folderName.str();
          std::filesystem::create_directories(methodDir);
          if (!WriteTextFile(methodDir / "summary.csv",
                             BuildSingleComparisonSummaryCsv(comparison)) ||
              !WriteTextFile(methodDir / "trajectory.csv",
                             BuildTrajectoryCsv(comparison.trajectory)) ||
              !WriteTextFile(methodDir / "trajectory-profile.csv",
                             BuildTrajectoryProfileCsv(solvedConfig, comparison.trajectory)) ||
              !WriteTextFile(methodDir / "speed-curve.csv",
                             BuildSpeedCurveCsv(comparison.speedCurve))) {
            errorMessage = std::string("Method export failed for: ") + comparison.name;
            return false;
          }
        }

        const std::filesystem::path attackDir = exportDir / "attack-trajectories";
        std::filesystem::create_directories(attackDir);
        for (std::size_t i = 0; i < result.attackTrajectories.size(); ++i) {
          const double headingDeg = i < result.attackTrajectoryHeadings.size()
                                        ? result.attackTrajectoryHeadings[i]
                                        : 0.0;
          std::ostringstream filename;
          filename << std::setw(2) << std::setfill('0') << (i + 1) << "-heading-"
                   << SlugifyFilename(FormatText("%.1f-deg", headingDeg)) << ".csv";
          if (!WriteTextFile(attackDir / filename.str(),
                             BuildTrajectoryCsv(result.attackTrajectories[i]))) {
            errorMessage = "Attack trajectory export failed.";
            return false;
          }
        }
      }

      if (result.targetSolution.enabled) {
        if (!WriteTextFile(exportDir / "target-trajectory.csv",
                           BuildTrajectoryCsv(result.targetSolution.trajectoryHermite)) ||
            !WriteTextFile(exportDir / "target-trajectory-profile.csv",
                           BuildTrajectoryProfileCsv(solvedConfig,
                                                     result.targetSolution.trajectoryHermite))) {
          errorMessage = "Target trajectory export failed.";
          return false;
        }
      }

      if (!ok) {
        errorMessage = "One or more export files could not be written.";
        return false;
      }
    } catch (const std::exception& ex) {
      errorMessage = ex.what();
      return false;
    }

    return true;
  }

  void drawNumericalNotes(const ballistics::LiveState& state, bool done) const {
    bool hasNotes = false;
    if (config_.vehicle.initialHeight < 10'000.0) {
      ImGui::SeparatorText("Numerical Notes");
      ImGui::TextColored(
          ImVec4(1.0f, 0.78f, 0.48f, 1.0f),
          "Low launch height: dense atmosphere can force much smaller steps and make "
          "attack-zone or target searches noticeably slower.");
      ImGui::TextDisabled("If the run appears stuck, check data/logs/osgearth_viewer.log.");
      hasNotes = true;
    }
    if (done && !state.finalResult.optimalTrajectory.impacted) {
      if (!hasNotes) {
        ImGui::SeparatorText("Numerical Notes");
        hasNotes = true;
      }
      ImGui::TextColored(
          ImVec4(1.0f, 0.62f, 0.48f, 1.0f),
          "The main trajectory did not produce a confirmed ground impact. Range or attack-zone "
          "data may be incomplete for this run.");
    }
    if (done && state.finalResult.targetSolution.enabled &&
        !state.finalResult.targetSolution.reachable) {
      if (!hasNotes) {
        ImGui::SeparatorText("Numerical Notes");
        hasNotes = true;
      }
      ImGui::TextColored(
          ImVec4(1.0f, 0.72f, 0.52f, 1.0f),
          "The requested target is outside the reachable set for the current initial "
          "conditions. The solver is showing the closest approach.");
    }
    if (state.status == ballistics::SolverStatus::Failed) {
      if (!hasNotes) {
        ImGui::SeparatorText("Numerical Notes");
        hasNotes = true;
      }
      ImGui::TextColored(
          ImVec4(1.0f, 0.55f, 0.48f, 1.0f),
          "A solver failure was recorded. Check data/logs/osgearth_viewer.log for the "
          "last completed phase and exception details.");
    }
  }

  void rebuildScene(const ballistics::LiveState& state, double animationProgress = 1.0) {
    dynamic_->removeChildren(0, dynamic_->getNumChildren());

    const auto& v = config_.vehicle;
    dynamic_->addChild(sphere(v.initialLongitudeDeg, v.initialLatitudeDeg, v.initialHeight,
                               16'000.0f, osg::Vec4f(0.24f, 0.96f, 0.74f, 0.95f)));
    dynamic_->addChild(line({world(v.initialLongitudeDeg, v.initialLatitudeDeg, 0.0),
                             world(v.initialLongitudeDeg, v.initialLatitudeDeg, v.initialHeight)},
                            osg::Vec4f(0.28f, 0.98f, 0.80f, 0.42f), 1.8f));
    char launchText[128];
    std::snprintf(launchText, sizeof(launchText), "Launch\n%.2f deg, %.2f deg\n%.1f km",
                  v.initialLongitudeDeg, v.initialLatitudeDeg, v.initialHeight / 1000.0);
    dynamic_->addChild(label(v.initialLongitudeDeg, v.initialLatitudeDeg, v.initialHeight,
                             launchText, 14, -18));

    const bool done = state.status == ballistics::SolverStatus::Done && state.hasFinalResult;
    const bool targetDone =
        done && mode_ == SolveMode::TargetPoint && state.finalResult.targetSolution.enabled;
    const auto animatedBest =
        done && !targetDone
            ? revealedTrajectory(state.finalResult.optimalHermitePoints, animationProgress)
            : std::vector<ballistics::PlotPoint>{};
    const auto& best =
        done ? (targetDone ? state.finalResult.optimalHermitePoints : animatedBest)
             : state.bestTrajectory;
    const std::vector<ballistics::PlotPoint> emptyCurrent;
    const auto& current = done ? emptyCurrent : state.currentTrajectory;

    if (state.attackBoundary.size() >= 3) {
      const auto outerLoop = boundaryLoop(state.attackBoundary, 0.05, 1'300.0);
      const auto ring75 = scaledBoundaryLoop(state.attackBoundary, 6, 0.75, 0.04, 1'150.0);
      const auto ring50 = scaledBoundaryLoop(state.attackBoundary, 6, 0.50, 0.03, 1'000.0);
      const auto ring25 = scaledBoundaryLoop(state.attackBoundary, 6, 0.25, 0.025, 900.0);
      const auto footprint = boundaryLoop(state.attackBoundary, 0.018, 750.0);
      dynamic_->addChild(attackZone(footprint, osg::Vec4f(0.03f, 0.58f, 0.88f, 0.08f)));
      dynamic_->addChild(line(path(ring25), osg::Vec4f(0.10f, 0.72f, 1.0f, 0.12f), 2.1f));
      dynamic_->addChild(line(path(ring50), osg::Vec4f(0.12f, 0.78f, 1.0f, 0.18f), 2.7f));
      dynamic_->addChild(line(path(ring75), osg::Vec4f(0.18f, 0.88f, 1.0f, 0.24f), 3.15f));
      dynamic_->addChild(line(path(footprint), osg::Vec4f(0.12f, 0.84f, 1.0f, 0.18f), 2.7f));
      dynamic_->addChild(line(path(outerLoop), osg::Vec4f(0.32f, 0.98f, 0.96f, 0.92f), 8.4f));
      addAttackZoneMarkers(state.attackBoundary);

      const std::size_t guideCount = std::min<std::size_t>(8, state.attackBoundary.size());
      for (std::size_t i = 0; i < guideCount; ++i) {
        const std::size_t idx =
            i * (outerLoop.size() - 1) / std::max<std::size_t>(1, guideCount);
        const double guideHeight =
            std::max(1'300.0, outerLoop[idx].height * 0.65);
        dynamic_->addChild(line(radialGuidePoints(outerLoop[idx], guideHeight),
                                osg::Vec4f(0.12f, 0.70f, 1.0f, 0.16f), 2.7f));
      }
    }

    for (std::size_t i = 0; i < state.attackTrajectories.size(); ++i) {
      const osg::Vec4f c = (i % 2 == 0) ? osg::Vec4f(0.12f, 0.98f, 1.0f, 0.32f)
                                        : osg::Vec4f(0.48f, 1.0f, 0.30f, 0.28f);
      dynamic_->addChild(line(path(state.attackTrajectories[i]), c, 5.85f));
    }

    if (best.size() >= 2) {
      constexpr double kBestPathBiasM = 6'000.0;
      dynamic_->addChild(line(surfaceProjectionPath(best, 1'400.0),
                              osg::Vec4f(0.14f, 0.86f, 1.0f, 0.78f), 4.8f));
      const osg::Vec4f bestGlow =
          targetDone ? osg::Vec4f(1.0f, 0.72f, 0.50f, 0.14f)
                     : osg::Vec4f(1.0f, 0.72f, 0.50f, 0.24f);
      const osg::Vec4f bestCore =
          targetDone ? osg::Vec4f(1.0f, 0.54f, 0.24f, 0.56f)
                     : osg::Vec4f(1.0f, 0.48f, 0.18f, 1.0f);
      const float bestGlowWidth = targetDone ? 28.0f : 42.0f;
      const float bestCoreWidth = targetDone ? 13.5f : 22.5f;
      dynamic_->addChild(line(path(best, kBestPathBiasM), bestGlow, bestGlowWidth, false));
      dynamic_->addChild(line(path(best, kBestPathBiasM), bestCore, bestCoreWidth, false));
      dynamic_->addChild(sphere(best.back().longitudeDeg, best.back().latitudeDeg, 0.0,
                                 14'000.0f, osg::Vec4f(1.0f, 0.40f, 0.20f, 0.96f)));
      char impactText[160];
      std::snprintf(impactText, sizeof(impactText), "Farthest Impact\n%.2f deg, %.2f deg",
                    best.back().longitudeDeg, best.back().latitudeDeg);
      dynamic_->addChild(label(best.back().longitudeDeg, best.back().latitudeDeg, 10'000.0,
                               impactText, 14, -14));
    }
    if (current.size() >= 2) {
      constexpr double kCurrentPathBiasM = 3'500.0;
      dynamic_->addChild(line(surfaceProjectionPath(current, 1'200.0),
                              osg::Vec4f(0.38f, 0.96f, 1.0f, 0.60f), 4.2f));
      dynamic_->addChild(line(path(current, kCurrentPathBiasM),
                              osg::Vec4f(1.0f, 0.96f, 0.50f, 0.22f), 33.0f, false));
      dynamic_->addChild(line(path(current, kCurrentPathBiasM),
                              osg::Vec4f(1.0f, 0.88f, 0.18f, 0.96f), 18.0f, false));
      dynamic_->addChild(sphere(current.back().longitudeDeg, current.back().latitudeDeg, 0.0,
                                 13'000.0f, osg::Vec4f(1.0f, 0.84f, 0.18f, 0.90f)));
    }

    if (done) {
      const auto& target = state.finalResult.targetSolution;
      if (target.enabled) {
        const osg::Vec4f tc = target.reachable ? osg::Vec4f(1.0f, 0.37f, 0.66f, 0.95f)
                                               : osg::Vec4f(1.0f, 0.54f, 0.30f, 0.95f);
        dynamic_->addChild(sphere(target.targetLongitudeDeg, target.targetLatitudeDeg,
                                   target.targetHeightM, 15'000.0f, tc));
        if (target.reachable && target.trajectoryHermite.size() >= 2) {
          const auto animatedTarget =
              targetDone ? revealedTrajectory(target.trajectoryHermite, animationProgress)
                         : target.trajectoryHermite;
          dynamic_->addChild(line(surfaceProjectionPath(animatedTarget, 1'300.0),
                                  osg::Vec4f(0.72f, 0.58f, 1.0f, 0.68f), 4.2f));
          dynamic_->addChild(line(path(animatedTarget, 4'500.0), tc, 18.0f, false));
        } else {
          dynamic_->addChild(sphere(target.closestLongitudeDeg, target.closestLatitudeDeg,
                                     target.closestHeightM, 12'000.0f,
                                     osg::Vec4f(1.0f, 0.82f, 0.36f, 0.95f)));
        }
      }

      const auto& focusPath =
          targetDone && state.finalResult.targetSolution.reachable
              ? state.finalResult.targetSolution.trajectoryHermite
              : state.finalResult.optimalHermitePoints;
      if (focus_ && !focusPath.empty() &&
          trajectoryAnimationVersion_ != focusedResultVersion_) {
        const auto& a = focusPath.front();
        const auto& b = focusPath.back();
        const double lon = 0.5 * (a.longitudeDeg + b.longitudeDeg);
        const double lat = 0.5 * (a.latitudeDeg + b.latitudeDeg);
        const double range = std::max(1.8e6, 1.15 * state.finalResult.optimalRange);
        manip_->setViewpoint(oe::Viewpoint("Result", lon, lat, 0.0, 0.0, -55.0, range), 1.3);
        focusedResultVersion_ = trajectoryAnimationVersion_;
      }
    }
  }

  void drawChart(const SeriesBundle& s) {
    auto maxOf = [](const std::vector<ballistics::PlotPoint>& pts, bool time) {
      double out = 1.0;
      for (const auto& p : pts) {
        out = std::max(out, time ? p.t : p.speed);
      }
      return out;
    };
    double maxT = 1.0;
    double maxV = 1.0;
    for (const auto& entry : s.entries) {
      maxT = std::max(maxT, maxOf(entry.points, true));
      maxV = std::max(maxV, maxOf(entry.points, false));
    }

    ImGui::BeginChild("chart", ImVec2(-1.0f, 252.0f), true,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove);
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 size(ImGui::GetContentRegionAvail().x, 238.0f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y),
                      IM_COL32(10, 16, 28, 185), 10.0f);
    const float plotLeft = origin.x + 56.0f;
    const float plotTop = origin.y + 26.0f;
    const float plotRight = origin.x + size.x - 14.0f;
    const float plotBottom = origin.y + size.y - 34.0f;
    const float plotWidth = std::max(1.0f, plotRight - plotLeft);
    const float plotHeight = std::max(1.0f, plotBottom - plotTop);
    dl->AddLine(ImVec2(plotLeft, plotTop), ImVec2(plotLeft, plotBottom),
                IM_COL32(170, 185, 210, 110), 1.2f);
    dl->AddLine(ImVec2(plotLeft, plotBottom), ImVec2(plotRight, plotBottom),
                IM_COL32(170, 185, 210, 110), 1.2f);

    for (int i = 0; i <= 4; ++i) {
      const float t = static_cast<float>(i) / 4.0f;
      const float y = plotBottom - t * plotHeight;
      const double value = maxV * t;
      dl->AddLine(ImVec2(plotLeft, y), ImVec2(plotRight, y),
                  IM_COL32(120, 150, 190, i == 0 ? 0 : 36), 1.0f);
      const std::string label = FormatText("%.0f", value);
      dl->AddText(ImVec2(origin.x + 10.0f, y - 8.0f), IM_COL32(206, 216, 230, 210),
                  label.c_str());
    }
    for (int i = 0; i <= 4; ++i) {
      const float t = static_cast<float>(i) / 4.0f;
      const float x = plotLeft + t * plotWidth;
      const double value = maxT * t;
      dl->AddLine(ImVec2(x, plotTop), ImVec2(x, plotBottom),
                  IM_COL32(120, 150, 190, i == 0 ? 0 : 28), 1.0f);
      const std::string label = FormatText("%.0f", value);
      dl->AddText(ImVec2(x - 8.0f, plotBottom + 8.0f), IM_COL32(206, 216, 230, 210),
                  label.c_str());
    }
    dl->AddText(ImVec2(plotLeft, origin.y + 6.0f), IM_COL32(210, 225, 242, 230), "Speed (m/s)");
    dl->AddText(ImVec2(plotRight - 64.0f, plotBottom + 20.0f), IM_COL32(210, 225, 242, 230),
                "Time (s)");

    const auto drawSeries = [&](const std::vector<ballistics::PlotPoint>& pts, ImU32 color,
                                float thickness) {
      if (pts.size() < 2) {
        return;
      }
      for (std::size_t i = 1; i < pts.size(); ++i) {
        const auto map = [&](const ballistics::PlotPoint& p) {
          return ImVec2(plotLeft + static_cast<float>((p.t / maxT) * plotWidth),
                        plotBottom - static_cast<float>((p.speed / maxV) * plotHeight));
        };
        dl->AddLine(map(pts[i - 1]), map(pts[i]), color, thickness);
      }
    };
    const float legendX = plotLeft + 10.0f;
    const float legendY = plotTop + 4.0f;
    for (std::size_t i = 0; i < s.entries.size(); ++i) {
      const auto& entry = s.entries[i];
      drawSeries(entry.points, entry.color, entry.thickness);
      const float y = legendY + static_cast<float>(i) * 18.0f;
      dl->AddLine(ImVec2(legendX, y + 6.0f), ImVec2(legendX + 20.0f, y + 6.0f),
                  entry.color, 3.0f);
      dl->AddText(ImVec2(legendX + 28.0f, y - 2.0f), IM_COL32(220, 228, 240, 220),
                  entry.name.c_str());
    }
    ImGui::Dummy(ImVec2(size.x, size.y));
    ImGui::EndChild();
  }

  void drawPanel(const ballistics::LiveState& state) {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 16.0f, vp->WorkPos.y + 16.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(460.0f, std::max(700.0f, vp->WorkSize.y - 32.0f)),
                             ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(360.0f, 440.0f),
                                        ImVec2(vp->WorkSize.x - 16.0f, vp->WorkSize.y - 16.0f));
    ImGui::SetNextWindowBgAlpha(0.90f);
    if (!ImGui::Begin("Ballistic Glide OSGEarth")) {
      ImGui::End();
      return;
    }

    ImGui::Text("Status: %s",
                state.status == ballistics::SolverStatus::Idle
                    ? "Idle"
                    : state.status == ballistics::SolverStatus::Running
                          ? "Running"
                          : state.status == ballistics::SolverStatus::Done ? "Done"
                                                                           : "Failed");
    ImGui::TextWrapped("%s", state.message.empty() ? "Waiting for computation." : state.message.c_str());
    if (!state.errorMessage.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.48f, 1.0f), "%s", state.errorMessage.c_str());
    }
    ImGui::ProgressBar(static_cast<float>(std::clamp(state.progress, 0.0, 1.0)),
                       ImVec2(-1.0f, 10.0f));

    ImGui::SeparatorText("Inputs");
    InputDouble("Launch longitude (deg)", &config_.vehicle.initialLongitudeDeg, 0.1, "%.4f");
    InputDouble("Launch latitude (deg)", &config_.vehicle.initialLatitudeDeg, 0.1, "%.4f");
    InputDouble("Launch height (m)", &config_.vehicle.initialHeight, 100.0, "%.1f");
    InputDouble("Initial speed (m/s)", &config_.vehicle.initialSpeed, 10.0, "%.2f");
    int alphaScheduleMode = static_cast<int>(config_.vehicle.alphaScheduleMode);
    const char* alphaScheduleItems[] = {"Baseline constant", "Smooth glide",
                                        "Qian-style", "Aggressive skip",
                                        "Qian skip-glide", "Sanger-style"};
    ImGui::Combo("Alpha schedule", &alphaScheduleMode, alphaScheduleItems,
                 IM_ARRAYSIZE(alphaScheduleItems));
    config_.vehicle.alphaScheduleMode =
        static_cast<ballistics::AlphaScheduleMode>(alphaScheduleMode);
    ImGui::SameLine();
    HelpMarker(AlphaScheduleHelp(config_.vehicle.alphaScheduleMode));
    InputDouble("Peak alpha (deg)", &config_.vehicle.initialAlphaDeg, 0.1, "%.2f");
    int solveMode = mode_ == SolveMode::Farthest ? 0 : 1;
    ImGui::RadioButton("1. Farthest range", &solveMode, 0);
    ImGui::RadioButton("2. Target point", &solveMode, 1);
    mode_ = solveMode == 0 ? SolveMode::Farthest : SolveMode::TargetPoint;
    if (mode_ == SolveMode::TargetPoint) {
      InputDouble("Target longitude (deg)", &config_.target.longitudeDeg, 0.1, "%.4f");
      InputDouble("Target latitude (deg)", &config_.target.latitudeDeg, 0.1, "%.4f");
      InputDouble("Target height (m)", &config_.target.heightM, 100.0, "%.1f");
    }
    ImGui::SeparatorText("Methods");
    ImGui::Checkbox("Fixed RK4 + linear", &config_.methods.fixedLinear);
    ImGui::Checkbox("Fixed RK4 + Hermite", &config_.methods.fixedHermite);
    ImGui::Checkbox("Adaptive RK4 + Hermite", &config_.methods.adaptiveHermite);
    ImGui::Checkbox("Genetic + adaptive", &config_.methods.geneticAdaptive);
    ImGui::TextDisabled(
        "Enable any subset. Main trajectory uses Adaptive > Fixed Hermite > Fixed Linear > Genetic among enabled methods.");
    if (ImGui::CollapsingHeader("More Runtime Parameters")) {
      InputDouble("Initial heading psi (deg)", &config_.vehicle.initialPsiDeg, 0.1, "%.3f");
      InputDouble("Mass (kg)", &config_.vehicle.mass, 1.0, "%.2f");
      InputDouble("Reference area", &config_.vehicle.refArea, 0.01, "%.3f");
      InputDouble("Min theta (deg)", &config_.search.minThetaDeg, 0.1, "%.3f");
      InputDouble("Max theta (deg)", &config_.search.maxThetaDeg, 0.1, "%.3f");
      InputDouble("Fixed step (s)", &config_.integration.fixedStep, 0.05, "%.3f");
      InputDouble("Adaptive step (s)", &config_.integration.adaptiveStep, 0.05, "%.3f");
      InputDouble("Target tolerance (m)", &config_.target.toleranceMeters, 100.0, "%.1f");
      ImGui::InputInt("GA population", &config_.search.geneticPopulation);
      ImGui::InputInt("GA elite count", &config_.search.geneticEliteCount);
      ImGui::InputInt("GA generations", &config_.search.geneticGenerationCount);
      ImGui::InputInt("GA random seed", &config_.search.geneticRandomSeed);
      InputDouble("GA mutation rate", &config_.search.geneticMutationRate, 0.01, "%.2f");
      InputDouble("GA mutation sigma (deg)", &config_.search.geneticMutationSigmaDeg, 0.1,
                  "%.2f");
      InputDouble("Live delay (ms)", &config_.output.liveDelayMs, 1.0, "%.1f");
    }
    ImGui::Checkbox("Animate live progress", &animate_);
    ImGui::SameLine();
    HelpMarker("Enabled: keep the live delay and animate the solver progress step by step.\nDisabled: remove the delay so the computation finishes as fast as possible.");
    ImGui::Checkbox("Auto focus result", &focus_);
    ImGui::SameLine();
    HelpMarker("Enabled: after the computation finishes, the camera automatically moves to frame the launch point and the computed result.");
    if (!hasEnabledMethods()) {
      ImGui::TextColored(ImVec4(1.0f, 0.60f, 0.48f, 1.0f),
                         "Enable at least one method before computing.");
    } else if (busy_.load()) {
      ImGui::TextColored(ImVec4(0.50f, 0.90f, 1.0f, 1.0f), "Solver is running...");
    } else if (ImGui::Button("Compute", ImVec2(-1.0f, 34.0f))) {
      startSolve();
    }

    const bool done = state.status == ballistics::SolverStatus::Done && state.hasFinalResult;
    drawNumericalNotes(state, done);

    ImGui::SeparatorText("Metrics");
    const double currentTheta = done ? state.finalResult.optimalThetaDeg : state.current.thetaDeg;
    const double currentPsi = done ? state.finalResult.optimalPsiDeg : state.current.psiDeg;
    const double currentRange = done ? state.finalResult.optimalRange / 1000.0 : state.current.range / 1000.0;
    const double bestTheta = done ? state.finalResult.optimalThetaDeg : state.best.thetaDeg;
    const double bestPsi = done ? state.finalResult.optimalPsiDeg : state.best.psiDeg;
    const double bestRange = done ? state.finalResult.optimalRange / 1000.0 : state.best.range / 1000.0;
    const double bestTime = done ? state.finalResult.optimalTrajectory.impactTime : state.best.flightTime;
    const double bestAlt = done ? state.finalResult.optimalTrajectory.maxAltitude / 1000.0
                                : state.best.maxAltitude / 1000.0;
    ImGui::TextDisabled("Current iteration and best solution are separated below.");
    if (ImGui::BeginTable("metrics-table", 2,
                          ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_BordersOuter |
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupColumn("Metric");
      ImGui::TableSetupColumn("Value");
      KeyValueRow("Current angle", FormatText("%.3f deg", currentTheta),
                  ImVec4(1.0f, 0.92f, 0.15f, 1.0f));
      KeyValueRow("Current heading", FormatText("%.3f deg", currentPsi),
                  ImVec4(1.0f, 0.92f, 0.15f, 1.0f));
      KeyValueRow("Current range", FormatText("%.3f km", currentRange),
                  ImVec4(1.0f, 0.92f, 0.15f, 1.0f));
      KeyValueRow("Best angle", FormatText("%.3f deg", bestTheta),
                  ImVec4(1.0f, 0.35f, 0.40f, 1.0f));
      KeyValueRow("Best heading", FormatText("%.3f deg", bestPsi),
                  ImVec4(1.0f, 0.35f, 0.40f, 1.0f));
      KeyValueRow("Best range", FormatText("%.3f km", bestRange),
                  ImVec4(1.0f, 0.35f, 0.40f, 1.0f));
      KeyValueRow("Flight time", FormatText("%.2f s", bestTime));
      KeyValueRow("Peak altitude", FormatText("%.2f km", bestAlt));
      ImGui::EndTable();
    }

    ImGui::SeparatorText("Computed Data");
    if (done) {
      const auto& finalResult = state.finalResult;
      ballistics::SolverConfig solvedConfig;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        solvedConfig = hasSolvedConfig_ ? solvedConfig_ : config_;
      }
      const auto impactPoint = finalResult.optimalHermitePoints.empty()
                                   ? ballistics::PlotPoint{}
                                   : finalResult.optimalHermitePoints.back();
      if (ImGui::BeginTable("computed-data-table", 2,
                            ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_BordersOuter |
                                ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Field");
        ImGui::TableSetupColumn("Value");
        KeyValueRow("Launch point",
                    FormatText("%.2f deg, %.2f deg, %.1f km",
                               solvedConfig.vehicle.initialLongitudeDeg,
                               solvedConfig.vehicle.initialLatitudeDeg,
                               solvedConfig.vehicle.initialHeight / 1000.0));
        KeyValueRow("Initial state",
                    FormatText("%.1f m/s | theta %.2f deg | psi %.2f deg",
                               solvedConfig.vehicle.initialSpeed,
                               solvedConfig.vehicle.initialThetaDeg,
                               solvedConfig.vehicle.initialPsiDeg));
        KeyValueRow("Alpha design",
                    FormatText("%s | peak %.2f deg",
                               AlphaScheduleName(solvedConfig.vehicle.alphaScheduleMode),
                               solvedConfig.vehicle.initialAlphaDeg));
        KeyValueRow("Optimal state",
                    FormatText("theta %.2f deg | psi %.2f deg",
                               finalResult.optimalThetaDeg,
                               finalResult.optimalPsiDeg));
        KeyValueRow("Enabled methods", EnabledMethodsText(solvedConfig));
        KeyValueRow("Impact point",
                    FormatText("%.2f deg, %.2f deg",
                               impactPoint.longitudeDeg,
                               impactPoint.latitudeDeg));
        KeyValueRow("Integration steps",
                    FormatText("%llu",
                               static_cast<unsigned long long>(
                                   finalResult.optimalTrajectory.acceptedSteps)));
        KeyValueRow("Max speed",
                    FormatText("%.2f m/s", finalResult.optimalTrajectory.maxSpeed));
        KeyValueRow("Valid attack headings",
                    FormatText("%llu / %d",
                               static_cast<unsigned long long>(
                                   finalResult.attackBoundary.size()),
                               solvedConfig.search.attackZoneHeadingCount));
        KeyValueRow("Trajectory samples",
                    FormatText("%llu",
                               static_cast<unsigned long long>(
                                   finalResult.optimalHermitePoints.size())));
        ImGui::EndTable();
      }
    } else {
      ImGui::TextWrapped("Computed data will appear here after the solver finishes.");
    }

    ImGui::SeparatorText("Target Solution");
    if (done && state.finalResult.targetSolution.enabled) {
      const auto& t = state.finalResult.targetSolution;
      const ImVec4 statusColor =
          t.reachable ? ImVec4(0.44f, 0.95f, 0.62f, 1.0f) : ImVec4(1.0f, 0.62f, 0.48f, 1.0f);
      if (ImGui::BeginTable("target-table", 2,
                            ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_BordersOuter |
                                ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Field");
        ImGui::TableSetupColumn("Value");
        KeyValueRow("Status", t.reachable ? "Reachable" : "Unreachable", statusColor);
        KeyValueRow("Target", FormatText("%.2f deg, %.2f deg, %.1f km", t.targetLongitudeDeg,
                                         t.targetLatitudeDeg, t.targetHeightM / 1000.0));
        KeyValueRow("Miss distance", FormatText("%.3f km", t.missDistance / 1000.0));
        KeyValueRow("Target angle", FormatText("%.3f deg", t.thetaDeg));
        KeyValueRow("Target heading", FormatText("%.3f deg", t.psiDeg));
        if (!t.reachable) {
          KeyValueRow("Closest point",
                      FormatText("%.2f deg, %.2f deg, %.1f km", t.closestLongitudeDeg,
                                 t.closestLatitudeDeg, t.closestHeightM / 1000.0));
          KeyValueRow("Closest time", FormatText("%.2f s", t.closestTime));
        } else {
          KeyValueRow("Flight time", FormatText("%.2f s", t.flightTime));
          KeyValueRow("Peak altitude", FormatText("%.2f km", t.peakAltitude / 1000.0));
        }
        ImGui::EndTable();
      }
    } else if (mode_ == SolveMode::Farthest) {
      ImGui::TextWrapped("Target solver is disabled in farthest-range mode.");
    } else {
      ImGui::TextWrapped("Target information will appear after the solver finishes.");
    }

    ImGui::SeparatorText("Speed-Time Curve");
    drawChart(series(state));

    ImGui::SeparatorText("Method Comparison");
    if (done && !state.finalResult.comparisons.empty()) {
      ImGui::TextDisabled(
          state.finalResult.targetSolution.enabled
              ? "Target-point mode: all rows below compare the matched target trajectory. The GA row uses a target-directed genetic search."
              : "Farthest-range mode: the result above comes from a full heading sweep, so different rows may now share the same globally farthest heading.");
      const float comparisonHeight =
          std::min(220.0f, ImGui::GetTextLineHeightWithSpacing() *
                                static_cast<float>(state.finalResult.comparisons.size() + 3));
      if (ImGui::BeginTable("method-comparison-table", 6,
                            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY |
                                ImGuiTableFlags_SizingFixedFit |
                                ImGuiTableFlags_Resizable,
                            ImVec2(-1.0f, comparisonHeight))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Method", ImGuiTableColumnFlags_WidthFixed, 190.0f);
        ImGui::TableSetupColumn("Angle", ImGuiTableColumnFlags_WidthFixed, 92.0f);
        ImGui::TableSetupColumn("Heading", ImGuiTableColumnFlags_WidthFixed, 98.0f);
        ImGui::TableSetupColumn("Range", ImGuiTableColumnFlags_WidthFixed, 102.0f);
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 96.0f);
        ImGui::TableSetupColumn("Steps", ImGuiTableColumnFlags_WidthFixed, 84.0f);
        ImGui::TableHeadersRow();
        for (const auto& row : state.finalResult.comparisons) {
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::TextUnformatted(row.name.c_str());
          ImGui::TableSetColumnIndex(1);
          ImGui::Text("%.3f deg", row.thetaDeg);
          ImGui::TableSetColumnIndex(2);
          ImGui::Text("%.3f deg", row.psiDeg);
          ImGui::TableSetColumnIndex(3);
          ImGui::Text("%.3f km", row.range / 1000.0);
          ImGui::TableSetColumnIndex(4);
          ImGui::Text("%.3f s", row.flightTime);
          ImGui::TableSetColumnIndex(5);
          ImGui::Text("%llu", static_cast<unsigned long long>(row.steps));
        }
        ImGui::EndTable();
      }
    } else {
      ImGui::TextWrapped("Fixed-step, adaptive-step, and interpolation comparisons will appear after the solver finishes.");
    }

    ImGui::SeparatorText("Export Bundle");
    if (done) {
      if (ImGui::Button("Export data bundle", ImVec2(-1.0f, 34.0f))) {
        std::filesystem::path exportDir;
        std::string exportError;
        if (exportBundle(state, exportDir, exportError)) {
          lastExportDirectory_ = exportDir;
          exportStatusMessage_ = "Exported: " + exportDir.string();
          exportFailed_ = false;
          ballistics::logInfo("osgearth_viewer",
                              std::string("Exported result bundle to ") +
                                  exportDir.string());
        } else {
          exportStatusMessage_ = "Export failed: " + exportError;
          exportFailed_ = true;
          ballistics::logError("osgearth_viewer", exportStatusMessage_);
        }
      }
      ImGui::TextDisabled(
          "Writes CSV data tables and a Markdown result sheet so you can draw the figures yourself.");
      if (!exportStatusMessage_.empty()) {
        const ImVec4 color =
            exportFailed_ ? ImVec4(1.0f, 0.58f, 0.48f, 1.0f)
                          : ImVec4(0.56f, 0.94f, 0.72f, 1.0f);
        ImGui::TextColored(color, "%s", exportStatusMessage_.c_str());
      }
    } else {
      ImGui::TextWrapped("Complete one solve first, then export the generated figures and data.");
    }
    ImGui::End();
  }

  osgViewer::Viewer* viewer_;
  oe::MapNode* mapNode_;
  oeu::EarthManipulator* manip_;
  osg::observer_ptr<osg::MatrixTransform> worldSpin_;
  osg::ref_ptr<const oe::SpatialReference> geoSrs_;
  ballistics::SolverConfig config_;
  ballistics::SolverConfig solvedConfig_;
  SolveMode mode_ = SolveMode::Farthest;
  bool animate_ = false;
  bool focus_ = true;
  bool hasSolvedConfig_ = false;
  bool exportFailed_ = false;
  std::mutex mutex_;
  ballistics::LiveState state_;
  std::uint64_t version_ = 0;
  std::uint64_t renderedVersion_ = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t trajectoryAnimationVersion_ = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t focusedResultVersion_ = std::numeric_limits<std::uint64_t>::max();
  std::filesystem::path lastExportDirectory_;
  std::string exportStatusMessage_;
  std::atomic<bool> busy_{false};
  std::chrono::steady_clock::time_point worldSpinStart_;
  std::chrono::steady_clock::time_point trajectoryAnimationStart_;
  double renderedAnimationProgress_ = -1.0;
  std::thread worker_;
  osg::ref_ptr<osg::Group> overlay_;
  osg::ref_ptr<osg::Group> static_;
  osg::ref_ptr<osg::Group> dynamic_;
};

std::filesystem::path earthPath(int argc, char** argv) {
  if (argc > 1) {
    return std::filesystem::path(argv[1]);
  }

  const std::filesystem::path appLocal =
      std::filesystem::current_path() / "assets" / "readymap.earth";
  if (std::filesystem::exists(appLocal)) {
    return appLocal;
  }

  return std::filesystem::current_path() / "apps" / "osgearth_viewer" / "assets" /
         "readymap.earth";
}

std::filesystem::path logFilePath() {
  return std::filesystem::current_path() / "data" / "logs" / "osgearth_viewer.log";
}

}  // namespace

int main(int argc, char** argv) {
  ballistics::initializeLogging(logFilePath().wstring());
  ballistics::installCrashHandlers();
  ballistics::logInfo("osgearth_viewer", "Application starting.");
  const std::filesystem::path path = earthPath(argc, argv);
  const std::filesystem::path earthDir = path.parent_path();

  _putenv_s("FONTCONFIG_PATH", "C:\\dev\\vcpkg\\installed\\x64-windows\\etc\\fonts");
  _putenv_s("FONTCONFIG_FILE",
            "C:\\dev\\vcpkg\\installed\\x64-windows\\etc\\fonts\\fonts.conf");

  osgDB::FilePathList dataPaths = osgDB::getDataFilePathList();
  if (!earthDir.empty()) {
    dataPaths.push_back(earthDir.string());
  }
  osgDB::setDataFilePathList(dataPaths);

  osg::ref_ptr<osg::Node> earth = osgDB::readRefNodeFile(path.string());
  if (!earth) {
    ballistics::logError("osgearth_viewer",
                         std::string("Failed to load earth file: ") + path.string());
    std::fprintf(stderr, "Failed to load earth file: %s\n", path.string().c_str());
    return 1;
  }

  oe::MapNode* mapNode = oe::MapNode::findMapNode(earth.get());
  if (!mapNode) {
    ballistics::logError("osgearth_viewer",
                         std::string("Failed to locate MapNode in: ") + path.string());
    std::fprintf(stderr, "Failed to locate MapNode in: %s\n", path.string().c_str());
    return 1;
  }

  osgViewer::Viewer viewer;
  viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
  viewer.setUpViewInWindow(80, 60, 1680, 940);
  viewer.getCamera()->setClearColor(osg::Vec4f(0.03f, 0.06f, 0.11f, 1.0f));
  viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
  oe::GLUtils::setGlobalDefaults(viewer.getCamera()->getOrCreateStateSet());
  viewer.setRealizeOperation(new oe::GL3RealizeOperation());

  auto manip = new oeu::EarthManipulator();
  manip->setHomeViewpoint(oe::Viewpoint("Home", 20.0, 38.0, 0.0, 0.0, -58.0, 2.4e6));
  viewer.setCameraManipulator(manip);

  auto worldSpin = new osg::MatrixTransform();
  worldSpin->addChild(earth.get());

  auto app = new BallisticApp(&viewer, mapNode, manip, worldSpin);
  worldSpin->addChild(app->overlayRoot());

  auto root = new osg::Group();
  root->addChild(worldSpin);
  viewer.setSceneData(root);
  viewer.addEventHandler(app);
  viewer.realize();
  manip->home(0.0);
  const int exitCode = viewer.run();
  ballistics::logInfo("osgearth_viewer",
                      std::string("Application exiting with code ") +
                          std::to_string(exitCode) + ".");
  return exitCode;
}
