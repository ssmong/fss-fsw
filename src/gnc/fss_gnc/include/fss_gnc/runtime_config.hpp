#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace fss::gnc {

inline double finite_number(const YAML::Node& value) {
    const double number = value.as<double>();
    if (!std::isfinite(number))
        throw std::invalid_argument("configuration values must be finite");
    return number;
}

template <std::size_t N>
std::array<double, N> finite_array(const YAML::Node& values) {
    if (!values.IsSequence() || values.size() != N)
        throw std::invalid_argument("configuration array has an invalid size");
    std::array<double, N> result{};
    for (std::size_t i = 0; i < N; ++i)
        result[i] = finite_number(values[i]);
    return result;
}

inline std::filesystem::path config_path(const std::filesystem::path& directory,
                                        std::string filename) {
    std::replace(filename.begin(), filename.end(), '\\', '/');
    return directory / std::filesystem::path(filename);
}

inline std::array<double, 8> load_thrust_limits(const std::filesystem::path& directory,
                                             const std::string& filename, double fallback) {
    std::array<double, 8> limits{};
    if (filename.empty()) {
        if (!std::isfinite(fallback) || fallback <= 0)
            throw std::invalid_argument("allocator.max_thrust_per_nozzle must be positive and finite");
        limits.fill(fallback);
        return limits;
    }

    const auto map = YAML::LoadFile(config_path(directory, filename).string());
    const auto duty = map["duty"];
    const auto thrust = map["thrust_n"];
    if (!duty.IsSequence() || duty.size() < 2 || !thrust.IsSequence() || thrust.size() != 8)
        throw std::invalid_argument("thrust map requires an input axis and eight thrust curves");
    if (finite_number(duty[0]) != 0 || finite_number(duty[duty.size() - 1]) != 1)
        throw std::invalid_argument("thrust map input axis must start at 0 and end at 1");
    for (std::size_t j = 1; j < duty.size(); ++j)
        if (finite_number(duty[j]) <= finite_number(duty[j - 1]))
            throw std::invalid_argument("thrust map input axis must be strictly increasing");
    for (std::size_t i = 0; i < 8; ++i) {
        const auto row = thrust[i];
        if (!row.IsSequence() || row.size() != duty.size() || finite_number(row[0]) != 0)
            throw std::invalid_argument("thrust curves must match the axis and start at zero");
        for (std::size_t j = 1; j < row.size(); ++j)
            if (finite_number(row[j]) < finite_number(row[j - 1]))
                throw std::invalid_argument("thrust curves must not decrease");
        limits[i] = finite_number(row[row.size() - 1]);
        if (limits[i] <= 0)
            throw std::invalid_argument("thrust map maximum must be positive");
    }
    return limits;
}

inline std::array<double, 24> load_allocation_matrix(
    const std::filesystem::path& directory, const std::array<double, 8>& limits) {
    const auto thrusters = YAML::LoadFile(
        (directory / "vehicle/thruster_geometry.yaml").string())["thruster_geometry"]["thrusters"];
    if (!thrusters.IsSequence() || thrusters.size() != 8)
        throw std::invalid_argument("thruster geometry must contain exactly eight thrusters");
    std::array<double, 24> matrix{};
    for (std::size_t i = 0; i < 8; ++i) {
        const auto position = finite_array<3>(thrusters[i]["position"]);
        const auto direction = finite_array<3>(thrusters[i]["direction"]);
        const double norm = std::hypot(direction[0], direction[1], direction[2]);
        if (std::abs(norm - 1.0) > 1.1e-5)
            throw std::invalid_argument("thruster direction must be a unit vector");
        matrix[i] = direction[0] * limits[i];
        matrix[8 + i] = direction[1] * limits[i];
        matrix[16 + i] = (position[0] * direction[1] - position[1] * direction[0]) * limits[i];
    }
    if (!std::all_of(matrix.begin(), matrix.end(), [](double value) { return std::isfinite(value); }))
        throw std::invalid_argument("allocation matrix must be finite");
    return matrix;
}

template <std::size_t N>
std::array<double, N> load_lqr_gains(const std::filesystem::path& directory, const char* key) {
    const auto gains = YAML::LoadFile((directory / "gnc/controller/lqr_gains.yaml").string());
    return finite_array<N>(gains["lqr"][key]);
}

}  // namespace fss::gnc
