#include "fss_navigation/ekf.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

// Text protocol keeps the numeric parity test independent of Python extension bindings.
int main() {
    using fss_navigation::PlanarEKF;
    fss_navigation::EKFParameters parameters{1.2217e-4, 5.886e-4, 1e-5, 1e-4, 5e-4, 1.745e-3};
    PlanarEKF filter(parameters);
    std::cout << std::setprecision(17);
    std::string operation;
    while (std::cin >> operation) {
        bool accepted = true;
        if (operation == "P") {
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
