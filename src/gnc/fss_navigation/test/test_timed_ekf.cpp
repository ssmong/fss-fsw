#include "fss_navigation/navigation.hpp"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <cmath>

namespace fss_navigation {
namespace {
constexpr double kPi = 3.14159265358979323846;
EKFParameters operational_parameters() {
    EKFParameters p{1.2217e-4, 3e-3, 1e-5, 1e-4, 5e-4, 5.235987756e-3};
    p.vicon_timestamp_std_ms = 5.0;
    p.accel_model_noise_density = 0.06;
    return p;
}

TEST(TimedEKF, DelayedMidIntervalUpdatesMatchChronologicalFilter) {
    PlanarEKF filter(operational_parameters()), chronological(operational_parameters());
    TimedEKF timed(filter, {}, 100000000);
    timed.initialize(0, Eigen::Vector3d::Zero());
    chronological.initialize(0, 0, 0);
    int previous = 0;
    for (int end = 10; end <= 200; end += 10) {
        timed.predict(end * 1000000LL, {0.4, 0.1, 0.0});
        int cursor = previous;
        for (int measurement : {15, 35, 55, 75, 95, 115}) {
            const double t = measurement / 1000.0;
            const Eigen::Vector3d pose(0.05*t*t, 0, 0.4*t);
            if (previous < measurement && measurement <= end) {
                chronological.predict((measurement-cursor)/1000.0, 0.4, 0.1, 0.0);
                ASSERT_TRUE(chronological.update_vicon(pose[0], pose[1], pose[2]));
                cursor = measurement;
            }
            if (end-10 < measurement+40 && measurement+40 <= end) {
                const auto result = timed.update(measurement*1000000LL, pose);
                ASSERT_TRUE(result.accepted);
                EXPECT_FALSE(result.recovered);
            }
        }
        chronological.predict((end-cursor)/1000.0, 0.4, 0.1, 0.0);
        previous = end;
    }
    EXPECT_LT((filter.x-chronological.x).norm(), 1e-12);
    EXPECT_LT((filter.P-chronological.P).norm(), 1e-12);
    EXPECT_EQ(timed.time_ns(), 200000000);
}

TEST(TimedEKF, HistoryBoundariesAndRejectedMeasurementsLeavePredictionUntouched) {
    PlanarEKF filter(operational_parameters());
    TimedEKF timed(filter, {}, 100000000);
    timed.initialize(0, Eigen::Vector3d::Zero());
    for (int ms=10; ms<=500; ms+=10) { timed.predict(ms*1000000LL, {0.2, 0.1, 0}); }
    const auto before_x = filter.x;
    const auto before_P = filter.P;
    const auto result = timed.update(495000000, {10, -10, 2});
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(timed.time_ns(), 500000000);
    EXPECT_TRUE(filter.x == before_x);
    EXPECT_TRUE(filter.P == before_P);
    for (auto stamp : {495000000, 100000000, 510000000}) {
        EXPECT_TRUE(timed.update(stamp, {1, 1, 1}).outside_history);
        EXPECT_TRUE(filter.x == before_x);
        EXPECT_TRUE(filter.P == before_P);
    }
    EXPECT_LE(timed.history_size(), 22u);
}

TEST(TimedEKF, ConsistentRejectedObservationsRecoverWithoutDiscardingBias) {
    for (int delay_ms : {0, 15}) {
        PlanarEKF filter(operational_parameters());
        TimedEKF timed(filter, {}, 100000000);
        timed.initialize(0, {0, 0, kPi-0.01});
        filter.P = PlanarEKF::Covariance::Identity()*1e-10;
        filter.x.tail<5>() << 0.001, 0.04, 0.03, 0, 0;
        const Eigen::Matrix<double,5,1> bias = filter.x.tail<5>();
        for (int n=1; n<=25; ++n) {
            const auto stamp = n*20000000LL;
            timed.predict(stamp, {0.001, 0.04, 0.03});
            timed.predict(stamp+delay_ms*1000000LL, {0.001, 0.04, 0.03});
            const auto result = timed.update(stamp, {0.04, 0, -kPi+0.01});
            EXPECT_EQ(result.recovered, n==25);
        }
        EXPECT_NEAR(filter.x[0], 0.04, 1e-12);
        EXPECT_TRUE(filter.x.tail<5>() == bias);
        EXPECT_DOUBLE_EQ(filter.nis_mean(), 0);
        EXPECT_GT(Eigen::SelfAdjointEigenSolver<PlanarEKF::Covariance>(filter.P).eigenvalues().minCoeff(), 0);
    }
}

TEST(TimedEKF, FrozenAndIncoherentMeasurementsCannotRecover) {
    for (int scenario=0; scenario<4; ++scenario) {
        PlanarEKF filter(operational_parameters());
        TimedEKF timed(filter, {}, 100000000);
        timed.initialize(0, {0, 0, scenario==0 ? 0.1 : 0});
        filter.P = PlanarEKF::Covariance::Identity()*1e-10;
        for (int n=1; n<=60; ++n) {
            const auto stamp = n*(scenario==3 ? 120000000LL : 20000000LL);
            timed.predict(stamp, {scenario==0 ? 0.3 : 0, 0, 0});
            const double x = scenario==1 ? 2.0 : scenario==2 ? (n%2 ? -0.1 : 0.1) : scenario==3 ? 0.1 : 0;
            const auto result = timed.update(stamp, {x, 0, 0});
            EXPECT_FALSE(result.recovered);
            if (scenario==0) { EXPECT_FALSE(result.accepted); }
        }
    }
}

TEST(TimedEKF, TimestampUncertaintyAcceptsMotionDirectionButRejectsLateralJump) {
    PlanarEKF filter(operational_parameters());
    TimedEKF timed(filter, {}, 100000000);
    timed.initialize(0, Eigen::Vector3d::Zero());
    const Eigen::Vector3d rates(1.2, 0.5, 2.4);
    filter.x.segment<2>(PlanarEKF::IVX) = rates.head<2>();
    for (int n=1; n<=600; ++n) {
        const auto stamp = n*10000000LL;
        timed.predict(stamp, {rates[2], 0, 0});
        if (n%2) { continue; }
        double t=n*0.01;
        if (350<n && n<=500) { t += n%4==0 ? .008 : -.008; }
        ASSERT_TRUE(timed.update(stamp, rates*t).accepted);
    }
    EXPECT_LT((filter.x.head<2>()-6*rates.head<2>()).norm(), .001);
    timed.predict(6020000000LL, {rates[2], 0, 0});
    EXPECT_FALSE(timed.update(6020000000LL, 6.02*rates+Eigen::Vector3d(-.01,.024,0)).accepted);
}

TEST(NavigationTiming, DelayedObservationAndFutureInputRespectSensorTimes) {
    auto timing = NavigationTiming{};
    timing.delay_ms=15;
    Navigation navigation(operational_parameters(),100,50,timing);
    EXPECT_EQ(navigation.on_vicon(100000000,100000000,0,0,0),InputResult::Accepted);
    EXPECT_EQ(navigation.on_imu(100000000,101000000,0,0,0),InputResult::Stale);
    navigation.on_imu(100000000,100000000,0,0,0);
    navigation.on_imu(110000000,110000000,0,0,0);
    ASSERT_TRUE(navigation.state(110000000));
    EXPECT_EQ(navigation.state(110000000)->stamp_ns,110000000);
    EXPECT_EQ(navigation.on_vicon(120000000,120000000,.0001,0,0),InputResult::Accepted);
    EXPECT_EQ(navigation.state(120000000)->stamp_ns,110000000);
    EXPECT_GT(navigation.filter().x[0],0);
}

TEST(NavigationTiming, FirstIMUAfterExpiredViconDoesNotIntegrateTheGap) {
    Navigation navigation(operational_parameters(),100,50);
    navigation.on_vicon(0,0,0,0,0);
    EXPECT_EQ(navigation.on_imu(200000000,200000000,0,1,0),InputResult::Resynchronizing);
    EXPECT_FALSE(navigation.state(200000000));
    EXPECT_DOUBLE_EQ(navigation.filter().x[PlanarEKF::IVX],0);
}

}  // namespace
}  // namespace fss_navigation
