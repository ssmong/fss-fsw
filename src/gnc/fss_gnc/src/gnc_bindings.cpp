// pybind11 바인딩 — C++ GNC 모듈을 Python에 노출
// 임포트: from fss_gnc._gnc_core import PIDTranslational, QPAllocator, ...

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <array>
#include <stdexcept>
#include <string>

#include "fss_gnc/types.hpp"
#include "fss_gnc/pid_controller.hpp"
#include "fss_gnc/lqr_controller.hpp"
#include "fss_gnc/qp_allocator.hpp"

namespace py = pybind11;
using namespace fss::gnc;

PYBIND11_MODULE(_gnc_core, m) {
    m.doc() = "FSS GNC C++ 코어 모듈 (PID/LQR/QP allocator)";

    // ── 타입 struct ──
    py::class_<TranslationalState>(m, "TranslationalState")
        .def(py::init<>())
        .def(py::init([](double x, double y, double vx, double vy, double theta) {
            return TranslationalState{x, y, vx, vy, theta};
        }), py::arg("x"), py::arg("y"), py::arg("vx"), py::arg("vy"), py::arg("theta") = 0.0)
        .def_readwrite("x", &TranslationalState::x)
        .def_readwrite("y", &TranslationalState::y)
        .def_readwrite("vx", &TranslationalState::vx)
        .def_readwrite("vy", &TranslationalState::vy)
        .def_readwrite("theta", &TranslationalState::theta);

    py::class_<TranslationalRef>(m, "TranslationalRef")
        .def(py::init<>())
        .def(py::init([](double x, double y, double vx, double vy) {
            return TranslationalRef{x, y, vx, vy};
        }), py::arg("x"), py::arg("y"), py::arg("vx"), py::arg("vy"))
        .def_readwrite("x", &TranslationalRef::x)
        .def_readwrite("y", &TranslationalRef::y)
        .def_readwrite("vx", &TranslationalRef::vx)
        .def_readwrite("vy", &TranslationalRef::vy);

    py::class_<RotationalState>(m, "RotationalState")
        .def(py::init<>())
        .def(py::init([](double theta, double omega) {
            return RotationalState{theta, omega};
        }), py::arg("theta"), py::arg("omega"))
        .def_readwrite("theta", &RotationalState::theta)
        .def_readwrite("omega", &RotationalState::omega);

    py::class_<RotationalRef>(m, "RotationalRef")
        .def(py::init<>())
        .def(py::init([](double theta, double omega) {
            return RotationalRef{theta, omega};
        }), py::arg("theta"), py::arg("omega"))
        .def_readwrite("theta", &RotationalRef::theta)
        .def_readwrite("omega", &RotationalRef::omega);

    py::class_<TranslationalOutput>(m, "TranslationalOutput")
        .def_readwrite("fx", &TranslationalOutput::fx)
        .def_readwrite("fy", &TranslationalOutput::fy);

    py::class_<AllocatorOutput>(m, "AllocatorOutput")
        .def("duty_list", [](const AllocatorOutput& o) {
            return std::vector<double>(o.duty, o.duty + 8);
        });

    // ── PID 제어기 ──
    py::class_<PIDTranslational>(m, "PIDTranslational")
        .def(py::init<>())
        .def("set_gains", &PIDTranslational::set_gains,
             py::arg("kp"), py::arg("ki"), py::arg("kd"), py::arg("max_force"))
        .def("compute", &PIDTranslational::compute,
             py::arg("state"), py::arg("ref"), py::arg("dt"))
        .def("reset", &PIDTranslational::reset);

    py::class_<PIDRotational>(m, "PIDRotational")
        .def(py::init<>())
        .def("set_gains", &PIDRotational::set_gains,
             py::arg("kp"), py::arg("ki"), py::arg("kd"), py::arg("max_torque"))
        .def("compute", &PIDRotational::compute,
             py::arg("state"), py::arg("ref"), py::arg("dt"))
        .def("reset", &PIDRotational::reset);

    // ── LQR 제어기 ──
    py::class_<LQRTranslational>(m, "LQRTranslational")
        .def(py::init<>())
        .def("set_gain", &LQRTranslational::set_gain,
             py::arg("K"), py::arg("max_force"))
        .def("compute", &LQRTranslational::compute,
             py::arg("state"), py::arg("ref"), py::arg("dt"))
        .def("reset", &LQRTranslational::reset);

    py::class_<LQRRotational>(m, "LQRRotational")
        .def(py::init<>())
        .def("set_gain", &LQRRotational::set_gain,
             py::arg("K"), py::arg("max_torque"))
        .def("compute", &LQRRotational::compute,
             py::arg("state"), py::arg("ref"), py::arg("dt"))
        .def("reset", &LQRRotational::reset);

    // ── QP 할당기 ──
    py::class_<QPAllocator>(m, "QPAllocator")
        .def(py::init<>())
        .def("set_B_matrix", [](QPAllocator& a, std::vector<double> b) {
            // 길이 불일치를 조용히 무시하면 B가 0으로 남아 무추력이 됨 → 예외로 승격
            if (b.size() != 24)
                throw std::invalid_argument(
                    "set_B_matrix: B 행렬은 3x8=24개 원소여야 함 (받은 크기: "
                    + std::to_string(b.size()) + ")");
            a.set_B_matrix(b.data());
        })
        .def_readwrite("thrust_weight", &QPAllocator::thrust_weight)
        .def_readwrite("min_thrust", &QPAllocator::min_thrust)
        .def_readwrite("max_thrust", &QPAllocator::max_thrust)
        .def_readwrite("max_iter", &QPAllocator::max_iter)
        .def("allocate", &QPAllocator::allocate,
             py::arg("fx"), py::arg("fy"), py::arg("mz"));
}
