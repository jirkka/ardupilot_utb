#include <AP_gtest.h>
#include <AP_UTB/AP_UTB.h>
#include <AP_UTB/AP_UTB_LogPacket.h>
#include <AP_UTB/AP_UTB_MotorMixer.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_UTB_ENABLED
// Host-only fixture transport. Production classes are linked without modifications.
TEST(UTBMath, OfflineVectors)
{
    const char *input_path = getenv("UTB_MATH_INPUT");
    const char *output_path = getenv("UTB_MATH_OUTPUT");
    if (input_path == nullptr || output_path == nullptr) {
        GTEST_SKIP() << "Run Tools/autotest/utb_math_validation.py to supply independent vectors";
    }
    FILE *input = fopen(input_path, "r");
    ASSERT_NE(input, nullptr);
    FILE *output = fopen(output_path, "wx");
    ASSERT_NE(output, nullptr);
    AP_UTB_RateController controller;
    AP_UTB_MotorMixer mixer;
    int scenario, sample, action, frame;
    float dt, thrust, desired[3], measured[3];
    AP_UTB_RateController::Gains gains[3];
    unsigned count = 0;
    while (fscanf(input, "%d %d %d %d %f %f %f %f %f %f %f %f",
                  &scenario, &sample, &action, &frame, &dt, &thrust,
                  &desired[0], &desired[1], &desired[2], &measured[0], &measured[1], &measured[2]) == 12) {
        for (auto &g : gains) {
            ASSERT_EQ(fscanf(input, "%f %f %f %f %f", &g.p, &g.i, &g.d, &g.imax, &g.d_hz), 5);
        }
        if (sample == 0 || action == 2) {
            controller.reset();
        }
        AP_UTB_State state;
        state.sampled_us = state.imu_updated_us = 1000000;
        state.attitude_body_to_ned = Quaternion(1, 0, 0, 0);
        state.attitude_valid = action != 3;
        state.rates_valid = true;
        state.rates_rads = Vector3f(measured[0], measured[1], measured[2]);
        ASSERT_TRUE(controller.configure(gains, 0.0025f));
        auto control = controller.update(state, Vector3f(desired[0], desired[1], desired[2]), dt);
        if (action == 1) {
            control = {};
            control.valid = true;
            control.command = Vector3f(desired[0], desired[1], desired[2]);
        }
        const auto allocation = mixer.mix(control, thrust, 1, frame);
        if (control.valid && allocation.valid && action != 1) {
            controller.feedback(control, allocation.achieved);
        }
        fprintf(output, "%d,%d,%u,%u,%u", scenario, sample, control.valid, control.priming, allocation.valid);
        for (const auto &a : control.axis) {
            fprintf(output, ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u",
                    double(a.error), double(a.p), double(a.i), double(a.d), double(a.raw),
                    double(a.output), double(a.acceleration), a.integration_blocked);
        }
        for (float motor : allocation.thrust) {
            fprintf(output, ",%.9g", double(motor));
        }
        fprintf(output, ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u\n",
                double(allocation.achieved.x), double(allocation.achieved.y), double(allocation.achieved.z),
                double(allocation.achieved_thrust), double(allocation.scale), double(allocation.collective_shift),
                allocation.positive_limits, allocation.negative_limits, allocation.lower_mask, allocation.upper_mask,
                allocation.moments_scaled, allocation.collective_shifted);
        count++;
    }
    EXPECT_GT(count, 1000U);
    EXPECT_EQ(fclose(input), 0);
    EXPECT_EQ(fclose(output), 0);
}

TEST(UTBMath, SustainedSaturationAndRecovery)
{
    for (float sign : {
             -1.0f, 1.0f
         }) {
        AP_UTB_RateController controller;
        AP_UTB_RateController::Gains gains[3] = {{1, 1, 0, 0.5f, 0}, {}, {}};
        ASSERT_TRUE(controller.configure(gains, 0.0025f));
        AP_UTB_State state;
        state.attitude_body_to_ned = Quaternion(1, 0, 0, 0);
        state.attitude_valid = state.rates_valid = true;
        controller.update(state, {}, 0.0025f);
        for (unsigned n = 0; n < 10000; n++) {
            const auto result = controller.update(state, {2 * sign, 0, 0}, 0.0025f);
            ASSERT_TRUE(result.valid);
            EXPECT_FLOAT_EQ(result.axis[0].i, sign * 0.005f);
            EXPECT_EQ(result.axis[0].integration_blocked, n != 0);
            const auto allocation = AP_UTB_MotorMixer().mix(result, 0.5f, 1, 18);
            ASSERT_TRUE(allocation.valid);
            controller.feedback(result, allocation.achieved);
        }
        const auto reversed = controller.update(state, {-sign, 0, 0}, 0.0025f);
        EXPECT_FALSE(reversed.axis[0].integration_blocked);
        EXPECT_FLOAT_EQ(reversed.axis[0].i, sign * 0.0025f);
        controller.reset();
        const auto prime = controller.update(state, {2 * sign, 0, 0}, 0.0025f);
        EXPECT_TRUE(prime.priming);
        EXPECT_FLOAT_EQ(prime.command.x, 0);
        EXPECT_FLOAT_EQ(prime.axis[0].i, 0);
    }
}

TEST(UTBMath, FormatRateAndBackendLossDoNotChangeMathematics)
{
    AP_UTB engine[2];
    AP_UTB_RateController::Gains gains[3] = {{0.3f, 0.2f, 0.004f, 0.2f, 20},
        {0.2f, 0.3f, 0.003f, 0.3f, 10}, {0.4f, 0.1f, 0.002f, 0.1f, 5}
    };
    AP_UTB::Inputs in;
    in.nominal_dt = 0.0025f;
    in.frame_class = 1;
    in.frame_type = 18;
    in.reference_valid = in.logging_available = true;
    in.state.attitude_body_to_ned = Quaternion(1, 0, 0, 0);
    in.state.attitude_valid = in.state.rates_valid = true;
    for (unsigned n = 0; n < 2000; n++) {
        in.dt = 0.0025f + (int(n % 17) - 8) * 0.000003f;
        in.time_us = 1000000 + n * 2500ULL;
        in.validation_time_us = in.state.sampled_us = in.state.imu_updated_us = uint32_t(in.time_us);
        in.reference = {float(n % 101) * 0.1f, -0.2f, 0.3f};
        in.state.rates_rads = {float(n % 13) * 0.001f, 0.02f, -0.03f};
        in.thrust = 0.5f;
        AP_UTB::Snapshot snapshot[2];
        for (unsigned variant = 0; variant < 2; variant++) {
            const int rate = variant == 0 ? 400 : (n < 1000 ? 200 : 50);
            ASSERT_TRUE(AP_Param::set_object_value(&engine[variant], AP_UTB::var_info, "LOG_FMT",
                                                   variant == 0 ? 1 : (n / 100) % 2));
            in.logging_available = variant == 0 || n < 500 || n >= 1500;
            engine[variant].evaluate(in, gains, snapshot[variant]);
            engine[variant].select_log_sample(in.time_us, in.nominal_dt, rate);
            int16_t words[UTBLogPacket::WORDS];
            UTBLogPacket::encode(snapshot[variant], words);
        }
        // Compare fields, never structure padding.
        for (unsigned axis = 0; axis < 3; axis++) {
            const auto &a = snapshot[0].control.axis[axis];
            const auto &b = snapshot[1].control.axis[axis];
            const float left[] = {a.error, a.p, a.i, a.d, a.raw, a.output, a.acceleration,
                                  snapshot[0].control.command[axis], snapshot[0].mixer.achieved[axis]
                                 };
            const float right[] = {b.error, b.p, b.i, b.d, b.raw, b.output, b.acceleration,
                                   snapshot[1].control.command[axis], snapshot[1].mixer.achieved[axis]
                                  };
            EXPECT_EQ(memcmp(left, right, sizeof(left)), 0);
            EXPECT_EQ(a.integration_blocked, b.integration_blocked);
        }
        const auto &a = snapshot[0].mixer;
        const auto &b = snapshot[1].mixer;
        const float left[] = {a.thrust[0], a.thrust[1], a.thrust[2], a.thrust[3],
                              a.achieved_thrust, a.scale, a.collective_shift
                             };
        const float right[] = {b.thrust[0], b.thrust[1], b.thrust[2], b.thrust[3],
                               b.achieved_thrust, b.scale, b.collective_shift
                              };
        EXPECT_EQ(memcmp(left, right, sizeof(left)), 0);
        EXPECT_EQ(a.valid, b.valid);
        EXPECT_EQ(a.positive_limits, b.positive_limits);
        EXPECT_EQ(a.negative_limits, b.negative_limits);
        EXPECT_EQ(a.lower_mask, b.lower_mask);
        EXPECT_EQ(a.upper_mask, b.upper_mask);
        EXPECT_EQ(a.moments_scaled, b.moments_scaled);
        EXPECT_EQ(a.collective_shifted, b.collective_shifted);
        EXPECT_EQ(snapshot[0].control.valid, snapshot[1].control.valid);
        EXPECT_EQ(snapshot[0].control.priming, snapshot[1].control.priming);
        EXPECT_EQ(snapshot[0].calculation, snapshot[1].calculation);
    }
}

TEST(UTBMath, HostWallBenchmark)
{
    AP_UTB_State state;
    state.attitude_body_to_ned = Quaternion(1, 0, 0, 0);
    state.attitude_valid = state.rates_valid = true;
    for (unsigned phase = 0; phase < 2; phase++) {
        for (unsigned active = 0; active < 2; active++) {
            AP_UTB_RateController controller;
            AP_UTB_RateController::Gains gains[3];
            for (auto &g : gains) {
                g = active ? AP_UTB_RateController::Gains(0.3f, 0.2f, 0.002f, 0.4f, 20) :
                    AP_UTB_RateController::Gains();
            }
            ASSERT_TRUE(controller.configure(gains, 0.0025f));
            controller.update(state, {}, 0.0025f);
            double checksum = 0;
            const auto start = std::chrono::steady_clock::now();
            for (unsigned n = 0; n < 1000000; n++) {
                state.rates_rads.x = float(n % 101) * 0.001f;
                const auto result = controller.update(state, {0.2f, -0.3f, 0.1f}, 0.0025f);
                if (phase == 0) {
                    checksum += result.command.x;
                } else {
                    const auto allocation = AP_UTB_MotorMixer().mix(result, 0.5f, 1, 18);
                    controller.feedback(result, allocation.achieved);
                    checksum += allocation.thrust[0];
                }
            }
            const auto end = std::chrono::steady_clock::now();
            printf("HOST_WALL phase=%u active=%u ns_per_cycle=%.3f checksum=%.9g\n", phase, active,
                   std::chrono::duration<double, std::nano>(end - start).count() / 1000000, checksum);
            EXPECT_TRUE(isfinite(checksum));
        }
    }
}
#endif // AP_UTB_ENABLED

AP_GTEST_MAIN()
