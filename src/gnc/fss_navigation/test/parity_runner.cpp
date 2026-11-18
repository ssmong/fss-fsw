#include "fss_navigation/ekf.hpp"
#include "fss_navigation/timed_ekf.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

// Text protocol keeps the numeric parity test independent of Python extension bindings.
int main(int argc, char** argv) {
    using fss_navigation::PlanarEKF;
    fss_navigation::EKFParameters parameters{1.2217e-4, 5.886e-4, 1e-5, 1e-4, 5e-4, 1.745e-3};
    const bool timed_mode = argc > 1 && std::string(argv[1]) == "--timed";
    if (timed_mode) {
        parameters.accel_noise_density = 3e-3;
        parameters.vicon_heading_noise = 5.235987756e-3;
        parameters.vicon_timestamp_std_ms = 5;
        parameters.accel_model_noise_density = .06;
    }
    PlanarEKF filter(parameters);
    fss_navigation::TimedEKF timed(filter, {}, 100000000);
    std::cout << std::setprecision(17);
    std::string operation;
    while (std::cin >> operation) {
        bool accepted = true;
        fss_navigation::TimedUpdate result;
        if (timed_mode && (operation == "I" || operation == "P" || operation == "V")) {
            std::int64_t stamp;
            double a, b, c;
            std::cin >> stamp >> a >> b >> c;
            if (operation == "I") { timed.initialize(stamp, {a, b, c}); }
            if (operation == "P") { timed.predict(stamp, {a, b, c}); }
            if (operation == "V") {
                result = timed.update(stamp, {a, b, c});
                accepted = result.accepted;
            }
        } else if (timed_mode && operation == "B") {
            filter.P = PlanarEKF::Covariance::Identity()*1e-10;
            for (int i=PlanarEKF::IBG; i<PlanarEKF::N_STATES; ++i) { std::cin >> filter.x[i]; }
        } else if (timed_mode && operation == "S") {
            std::cin >> filter.x[PlanarEKF::IVX] >> filter.x[PlanarEKF::IVY];
        } else if (operation == "P") {
            double dt, omega, ax, ay;
            std::cin >> dt >> omega >> ax >> ay;
            filter.predict(dt, omega, ax, ay);
        } else if (operation == "V") {
            double px, py, theta;
            std::cin >> px >> py >> theta;
            accepted = filter.update_vicon(px, py, theta);
        } else if (operation == "I") {
            double px, py, theta;
            std::cin >> px >> py >> theta;
            filter.initialize(px, py, theta);
        } else {
            throw std::invalid_argument("Unknown parity operation");
        }
        if (!std::cin) {
            return 1;
        }
        std::cout << accepted << ' ' << filter.filter_consistent() << ' ' << filter.nis_mean();
        if (timed_mode) {
            std::cout << ' ' << result.recovered << ' ' << result.outside_history << ' ' << result.nis;
            for (int i=0; i<3; ++i) { std::cout << ' ' << result.innovation[i]; }
        }
        for (int i = 0; i < PlanarEKF::N_STATES; ++i) {
            std::cout << ' ' << filter.x[i];
        }
        for (int row = 0; row < PlanarEKF::N_STATES; ++row) {
            for (int column = 0; column < PlanarEKF::N_STATES; ++column) {
                std::cout << ' ' << filter.P(row, column);
            }
        }
        std::cout << '\n';
    }
}
