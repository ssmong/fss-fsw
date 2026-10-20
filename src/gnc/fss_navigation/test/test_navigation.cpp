#include "fss_navigation/navigation.hpp"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace fss_navigation {
namespace {
constexpr double kPi = 3.14159265358979323846;
const EKFParameters kParameters{1.2217e-4, 5.886e-4, 1e-5, 1e-4, 5e-4, 1.745e-3};

TEST(PlanarEKF, InitializationAndInvalidInputs) {
    PlanarEKF filter(kParameters);
    filter.predict(0.01, 1.0, 1.0, 1.0);
    EXPECT_TRUE(filter.x.isZero());
    EXPECT_FALSE(filter.initialized());
    EXPECT_FALSE(filter.update_vicon(NAN, 0.0, 0.0));
    ASSERT_TRUE(filter.update_vicon(1.0, 2.0, 0.5));
    EXPECT_EQ(filter.x.size(), 11);
    EXPECT_DOUBLE_EQ(filter.x[PlanarEKF::IPX], 1.0);
    EXPECT_DOUBLE_EQ(filter.x[PlanarEKF::IPY], 2.0);
    EXPECT_DOUBLE_EQ(filter.x[PlanarEKF::ITHETA], 0.5);
    EXPECT_TRUE(filter.x.tail<5>().isZero());
    EXPECT_DOUBLE_EQ(filter.nis_mean(), 0.0);
    EXPECT_DOUBLE_EQ(filter.P(PlanarEKF::IBTX, PlanarEKF::IBTX), 2.5e-3);
    const auto x = filter.x;
    const auto P = filter.P;
    for (double dt : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity()}) {
        filter.predict(dt, 1.0, 1.0, 1.0);
    }
    filter.predict(0.01, NAN, 1.0, 1.0);
    EXPECT_TRUE(filter.x == x);
    EXPECT_TRUE(filter.P == P);
}

TEST(PlanarEKF, StationaryPredictionGrowsCovarianceAndViconUpdateShrinksIt) {
    PlanarEKF filter(kParameters);
    filter.initialize(1.0, 2.0, 0.0);
    const double initial_variance = filter.P(PlanarEKF::IPX, PlanarEKF::IPX);
    for (int i = 0; i < 100; ++i) {
        filter.predict(0.01, 0.0, 0.0, 0.0);
    }
    EXPECT_DOUBLE_EQ(filter.x[PlanarEKF::IPX], 1.0);
    EXPECT_DOUBLE_EQ(filter.x[PlanarEKF::IPY], 2.0);
    const double predicted_variance = filter.P(PlanarEKF::IPX, PlanarEKF::IPX);
    EXPECT_GT(predicted_variance, initial_variance);
    ASSERT_TRUE(filter.update_vicon(1.001, 2.001, 0.001));
    EXPECT_LT(filter.P(PlanarEKF::IPX, PlanarEKF::IPX), predicted_variance);
    EXPECT_GT(filter.nis_mean(), 0.0);
}

TEST(PlanarEKF, GyroBiasConvergesWithinFiveSeconds) {
    PlanarEKF filter(kParameters);
    filter.initialize(0.0, 0.0, 0.0);
    for (int i = 0; i < 500; ++i) {
        filter.predict(0.01, 0.005, 0.0, 0.0);
        if (i % 5 == 0) {
            ASSERT_TRUE(filter.update_vicon(0.0, 0.0, 0.0));
        }
    }
    EXPECT_NEAR(filter.x[PlanarEKF::IBG], 0.005, 0.001);
}

TEST(PlanarEKF, MidpointRotationAndInertialVelocity) {
    PlanarEKF filter(kParameters);
    filter.initialize(0.0, 0.0, kPi / 2.0);
    for (int i = 0; i < 100; ++i) {
        filter.predict(0.01, 0.0, 0.1, 0.0);
    }
    EXPECT_NEAR(filter.x[PlanarEKF::IPX], 0.0, 1e-12);
    EXPECT_NEAR(filter.x[PlanarEKF::IPY], 0.05, 1e-12);
    EXPECT_NEAR(filter.x[PlanarEKF::IVY], 0.1, 1e-12);
    filter.initialize(0.0, 0.0, 0.0);
    filter.x[PlanarEKF::IVX] = 0.2;
    filter.x[PlanarEKF::IBG] = 0.01;
    for (int i = 0; i < 200; ++i) {
        filter.predict(0.01, 0.51, 0.0, 0.0);
    }
    EXPECT_NEAR(filter.x[PlanarEKF::IPX], 0.4, 1e-12);
    EXPECT_DOUBLE_EQ(filter.x[PlanarEKF::IVX], 0.2);
    EXPECT_DOUBLE_EQ(filter.x[PlanarEKF::IOMEGA], 0.5);
}

TEST(PlanarEKF, GatingWrappedInnovationAndNISWindow) {
    auto parameters = kParameters;
    parameters.nis_window = 10;
    PlanarEKF filter(parameters);
    filter.initialize(0.0, 0.0, kPi - 0.001);
    ASSERT_TRUE(filter.update_vicon(0.0, 0.0, -kPi + 0.001));
    const auto x = filter.x;
    const auto P = filter.P;
    EXPECT_FALSE(filter.update_vicon(10.0, 10.0, 0.0));
    EXPECT_TRUE(filter.x == x);
    EXPECT_TRUE(filter.P == P);
    EXPECT_GT(filter.nis_mean(), 0.0);
    for (int i = 0; i < 10; ++i) {
        EXPECT_TRUE(filter.update_vicon(filter.x[0], filter.x[1], filter.x[2]));
    }
    EXPECT_DOUBLE_EQ(filter.nis_mean(), 0.0);
    EXPECT_FALSE(filter.filter_consistent());
}

TEST(PlanarEKF, BiasConvergenceAndPositiveCovariance) {
    PlanarEKF filter(kParameters);
    filter.initialize(0.0, 0.0, 0.0);
    double theta = 0.0;
    double px = 0.0;
    double vx = 0.0;
    for (int i = 0; i < 5000; ++i) {
        theta += 0.3 * 0.01;
        px += vx * 0.01 + 0.5 * 0.02 * 0.01 * 0.01;
        vx += 0.02 * 0.01;
        filter.predict(0.01, 0.305, 0.008, 0.0);
        if (i % 5 == 0) {
            ASSERT_TRUE(filter.update_vicon(px, 0.0, theta));
        }
    }
    EXPECT_NEAR(filter.x[PlanarEKF::IBG], 0.005, 0.001);
    EXPECT_NEAR(filter.x[PlanarEKF::IBAX], 0.008, 0.003);
    EXPECT_NEAR(filter.x[PlanarEKF::IBTX], 0.02, 0.005);
    EXPECT_NEAR(filter.x[PlanarEKF::IBAY], 0.0, 0.003);
    EXPECT_NEAR(filter.x[PlanarEKF::IBTY], 0.0, 0.005);
    EXPECT_LT((filter.P - filter.P.transpose()).norm(), 1e-14);
    const Eigen::SelfAdjointEigenSolver<PlanarEKF::Covariance> eigenvalues(filter.P);
    EXPECT_GT(eigenvalues.eigenvalues().minCoeff(), -1e-12);
}

TEST(PlanarEKF, IsolatedBodyAndTableBiasesMeetOriginalConvergenceLimits) {
    for (bool table_tilt : {false, true}) {
        SCOPED_TRACE(table_tilt ? "table tilt" : "body accelerometer bias");
        PlanarEKF filter(kParameters);
        filter.initialize(0.0, 0.0, 0.0);
        const double omega = table_tilt ? 0.3 : 0.4;
        double theta = 0.0;
        double px = 0.0;
        double vx = 0.0;
        for (int i = 0; i < (table_tilt ? 5000 : 4000); ++i) {
            theta += omega * 0.01;
            if (table_tilt) {
                px += vx * 0.01 + 0.5 * 0.02 * 0.01 * 0.01;
                vx += 0.02 * 0.01;
            }
            filter.predict(0.01, omega, table_tilt ? 0.0 : 0.008, 0.0);
            if (i % 5 == 0) {
                ASSERT_TRUE(filter.update_vicon(px, 0.0, theta));
            }
        }
        const double tolerance = table_tilt ? 0.005 : 0.003;
        EXPECT_NEAR(filter.x[PlanarEKF::IBAX], table_tilt ? 0.0 : 0.008, tolerance);
        EXPECT_NEAR(filter.x[PlanarEKF::IBTX], table_tilt ? 0.02 : 0.0, tolerance);
    }
}

TEST(PlanarEKF, CovarianceFailureRemainsVisible) {
    EXPECT_DOUBLE_EQ(covariance_to_std(4.0), 2.0);
    EXPECT_DOUBLE_EQ(covariance_to_std(0.0), 0.0);
    EXPECT_DOUBLE_EQ(covariance_to_std(-1e-12), 0.0);
    EXPECT_TRUE(std::isinf(covariance_to_std(NAN)));
    EXPECT_TRUE(std::isinf(covariance_to_std(INFINITY)));
    PlanarEKF filter(kParameters);
    filter.initialize(0.0, 0.0, 0.0);
    filter.P(0, 0) = NAN;
    EXPECT_FALSE(filter.update_vicon(0.0, 0.0, 0.0));
    EXPECT_TRUE(std::isinf(filter.nis_mean()));
    EXPECT_TRUE(std::isnan(filter.P(0, 0)));
}

TEST(Navigation, BurstSamplesUseSensorIntervalsAndIgnoreReorderedInputs) {
    Navigation navigation(kParameters, 100.0, 50);
    navigation.on_vicon(0, 0, 0.0, 0.0, 0.0);
    navigation.on_imu(30000000, 10000000, 0.0, 0.0, 0.0);
    navigation.on_imu(30000000, 20000000, 0.0, 1.0, 0.0);
    EXPECT_EQ(navigation.on_imu(30000000, 20000000, 0.0, 100.0, 0.0), InputResult::Reordered);
    EXPECT_EQ(navigation.on_imu(30000000, 15000000, 0.0, 100.0, 0.0), InputResult::Reordered);
    navigation.on_imu(30000000, 30000000, 0.0, 1.0, 0.0);
    const auto state = navigation.state(30000000);
    ASSERT_TRUE(state);
    EXPECT_NEAR(state->x[PlanarEKF::IVX], 0.02, 1e-14);
    EXPECT_EQ(state->stamp_ns, 30000000);
}

TEST(Navigation, InvalidUnavailableStaleAndFutureSamplesCannotRefreshState) {
    Navigation navigation(kParameters, 100.0, 50);
    navigation.on_vicon(0, 0, 0.0, 0.0, 0.0);
    navigation.on_imu(0, 0, 0.0, 0.0, 0.0);
    EXPECT_EQ(navigation.on_imu(10000000, 10000000, 0.0, 1.0, 0.0, false), InputResult::Invalid);
    EXPECT_EQ(navigation.on_imu(10000000, 10000000, 0.0, 1.0, 0.0, true, false), InputResult::Invalid);
    EXPECT_EQ(navigation.on_imu(10000000, 10000000, NAN, 1.0, 0.0), InputResult::Invalid);
    EXPECT_EQ(navigation.on_imu(1000000000, 0, 0.0, 1.0, 0.0), InputResult::Stale);
    EXPECT_EQ(navigation.on_imu(1000000000, 2000000000, 0.0, 1.0, 0.0), InputResult::Stale);
    EXPECT_FALSE(navigation.state(10000000));
    EXPECT_DOUBLE_EQ(navigation.filter().x[PlanarEKF::IVX], 0.0);
    navigation.on_imu(20000000, 20000000, 0.0, 1.0, 0.0);
    EXPECT_NEAR(navigation.filter().x[PlanarEKF::IVX], 0.02, 1e-14);
}

TEST(Navigation, IMUOnlyAndDuplicateViconDoNotPublishOrTightenCovariance) {
    Navigation navigation(kParameters, 100.0, 50);
    navigation.on_imu(0, 0, 0.0, 0.0, 0.0);
    navigation.on_imu(10000000, 10000000, 0.0, 0.0, 0.0);
    EXPECT_FALSE(navigation.state(10000000));
    navigation.on_vicon(10000000, 10000000, 0.0, 0.0, 0.0);
    const auto P = navigation.filter().P;
    EXPECT_EQ(navigation.on_vicon(20000000, 10000000, 0.0, 0.0, 0.0), InputResult::Reordered);
    EXPECT_TRUE(P == navigation.filter().P);
    EXPECT_FALSE(navigation.state(20000000));
}

TEST(Navigation, OutageRequiresFreshViconAndTwoIMUSamplesToResume) {
    Navigation navigation(kParameters, 100.0, 50);
    navigation.on_vicon(0, 0, 0.0, 0.0, 0.0);
    navigation.on_imu(0, 0, 0.0, 0.0, 0.0);
    navigation.on_imu(10000000, 10000000, 0.0, 0.0, 0.0);
    ASSERT_TRUE(navigation.state(10000000));
    EXPECT_FALSE(navigation.state(300000000));
    EXPECT_TRUE(navigation.needs_vicon_initialization());
    EXPECT_EQ(navigation.on_vicon(300000000, 0, 1.0, 0.0, 0.0), InputResult::Stale);
    navigation.on_imu(300000000, 300000000, 0.0, 1.0, 0.0);
    EXPECT_FALSE(navigation.state(300000000));
    ASSERT_EQ(navigation.on_vicon(300000000, 300000000, 1.0, 0.0, 0.0), InputResult::Accepted);
    EXPECT_FALSE(navigation.state(300000000));
    navigation.on_imu(300000000, 300000000, 0.0, 0.0, 0.0);
    EXPECT_FALSE(navigation.state(300000000));
    navigation.on_imu(310000000, 310000000, 0.0, 0.0, 0.0);
    const auto state = navigation.state(310000000);
    ASSERT_TRUE(state);
    EXPECT_DOUBLE_EQ(state->x[0], 1.0);
}

TEST(Navigation, GapBeforeFirstPredictionAndBeforeTimerCannotIntegrateMissingMotion) {
    for (bool predicted : {false, true}) {
        Navigation navigation(kParameters, 100.0, 50);
        navigation.on_vicon(0, 0, 0.0, 0.0, 0.0);
        navigation.on_imu(0, 0, 0.0, 0.0, 0.0);
        if (predicted) {
            navigation.on_imu(10000000, 10000000, 0.0, 0.0, 0.0);
        }
        EXPECT_EQ(navigation.on_imu(300000000, 300000000, 0.0, 1.0, 0.0),
                  InputResult::Resynchronizing);
        EXPECT_FALSE(navigation.state(300000000));
        EXPECT_TRUE(navigation.needs_vicon_initialization());
        EXPECT_DOUBLE_EQ(navigation.filter().x[PlanarEKF::IVX], 0.0);
    }
}

TEST(Navigation, ViconOutlierCannotResetTimeoutAndFreshViconRestoresNavigation) {
    Navigation navigation(kParameters, 100.0, 2);
    navigation.on_vicon(0, 0, 0.0, 0.0, 0.0);
    navigation.on_imu(0, 0, 0.0, 0.0, 0.0);
    for (int i = 1; i <= 14; ++i) {
        const auto time = i * 10000000LL;
        navigation.on_imu(time, time, 0.0, 0.0, 0.0);
        if (i == 12) {
            EXPECT_EQ(navigation.on_vicon(time, time, 10.0, 10.0, 0.0), InputResult::Rejected);
        }
        ASSERT_TRUE(navigation.state(time));
    }
    EXPECT_TRUE(navigation.navigation_degraded());
    EXPECT_EQ(navigation.on_vicon(150000000, 150000000, 0.0, 0.0, 0.0), InputResult::Accepted);
    EXPECT_FALSE(navigation.navigation_degraded());
}

TEST(Navigation, ClockReversalSuspendsPublicationAndTimeoutBoundaryIsInclusive) {
    Navigation navigation(kParameters, 100.0, 50);
    navigation.on_vicon(0, 0, 0.0, 0.0, 0.0);
    navigation.on_imu(0, 0, 0.0, 0.0, 0.0);
    navigation.on_imu(10000000, 10000000, 0.0, 0.0, 0.0);
    ASSERT_TRUE(navigation.state(110000000));
    EXPECT_FALSE(navigation.state(9000000));
    EXPECT_TRUE(navigation.needs_vicon_initialization());
}

}  // namespace
}  // namespace fss_navigation
