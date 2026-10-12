#include <AP_gtest.h>
#include <AP_UTB/AP_UTB_SyntheticReference.h>
#include <cstdio>
#include <cstdlib>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
#if AP_UTB_ENABLED && AP_UTB_BENCH_ENABLED
using Generator = AP_UTB_SyntheticReference;
TEST(UTBSynthetic, OfflineReferenceVectors)
{
    const char *path = getenv("UTB_REFERENCE_OUTPUT");
    if (path == nullptr) {
        GTEST_SKIP() << "Supply UTB_REFERENCE_OUTPUT for independent signal comparison";
    }
    FILE *output = fopen(path, "wx");
    ASSERT_NE(output, nullptr);
    for (uint8_t profile = 1; profile <= 2; profile++) {
        for (uint32_t cycle = 0; cycle < Generator::CYCLES; cycle++) {
            const auto rates = Generator::signal(cycle, profile);
            fprintf(output, "%u,%u,%u,%.9g,%.9g,%.9g\n", profile, cycle, Generator::phase(cycle),
                    double(rates.x), double(rates.y), double(rates.z));
        }
    }
    EXPECT_EQ(fclose(output), 0);
}
TEST(UTBSynthetic, CompleteFiniteSequenceAndLatch)
{
    Generator generator;
    EXPECT_EQ(generator.update(1, false, false, false, 0).status, Generator::Status::WAITING);
    for (unsigned n = 0; n < 400; n++) {
        const auto sample = generator.update(1, true, false, false, 0);
        EXPECT_EQ(sample.status, Generator::Status::PREROLL);
        EXPECT_TRUE(sample.valid);
        EXPECT_FLOAT_EQ(sample.rates.length(), 0);
    }
    for (uint32_t n = 0; n < Generator::CYCLES; n++) {
        const auto sample = generator.update(1, true, false, false, 0);
        ASSERT_TRUE(sample.valid);
        EXPECT_EQ(sample.cycle, n);
        EXPECT_EQ(sample.phase, Generator::phase(n));
        for (uint8_t j = 0; j < 3; j++) {
            EXPECT_LE(fabsf(sample.rates[j]), 0.3f);
        }
        if (n < 200 || n >= 7400) {
            EXPECT_FLOAT_EQ(sample.rates.length(), 0);
        }
    }
    EXPECT_EQ(generator.update(1, true, false, false, 0).status, Generator::Status::COMPLETE);
    EXPECT_FALSE(generator.update(1, true, false, false, 0).valid);
    EXPECT_EQ(generator.update(0, true, false, false, 0).status, Generator::Status::OFF);
    EXPECT_EQ(generator.update(1, true, false, false, 0).cycle, 0U);
}

TEST(UTBSynthetic, FaultsDoNotAutomaticallyResume)
{
    for (unsigned fault = 0; fault < 5; fault++) {
        Generator generator;
        ASSERT_TRUE(generator.update(1, true, false, false, 0).valid);
        const auto sample = generator.update(fault == 4 ? 2 : 1, fault != 0, fault == 1,
                                             fault == 2, fault == 3 ? 1 : 0);
        EXPECT_EQ(sample.status, Generator::Status::ABORTED);
        EXPECT_FALSE(sample.valid);
        EXPECT_FLOAT_EQ(sample.rates.length(), 0);
        EXPECT_FALSE(generator.update(1, true, false, false, 0).valid);
        generator.update(0, false, false, false, 0);
        EXPECT_TRUE(generator.update(1, true, false, false, 0).valid);
    }
}

TEST(UTBSynthetic, AnalyticalPointsAndSaturationProfile)
{
    EXPECT_FLOAT_EQ(Generator::signal(200, 1).x, 0.3f);
    EXPECT_FLOAT_EQ(Generator::signal(1800, 1).x, -0.3f);
    EXPECT_FLOAT_EQ(Generator::signal(3400, 1).y, 0.3f);
    EXPECT_FLOAT_EQ(Generator::signal(4200, 1).y, -0.3f);
    EXPECT_NEAR(Generator::signal(5200, 1).x, 0.3f, 2 * FLT_EPSILON);
    EXPECT_NEAR(Generator::signal(6000, 1).z, 0.3f, 2 * FLT_EPSILON);
    EXPECT_FLOAT_EQ(Generator::signal(8000, 1).length(), 0);
    Generator generator;
    EXPECT_EQ(generator.update(3, true, false, false, 0).status, Generator::Status::ABORTED);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    Generator saturation;
    EXPECT_TRUE(saturation.update(2, true, false, false, 0).valid);
#endif
}
#endif
AP_GTEST_MAIN()
