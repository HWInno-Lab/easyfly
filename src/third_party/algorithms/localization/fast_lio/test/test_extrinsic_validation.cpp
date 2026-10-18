#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "extrinsic_validation.hpp"

TEST(ExtrinsicValidation, AcceptsExpectedFiniteArrays)
{
    const std::vector<double> translation{0.0, 0.0, 0.0};
    const std::vector<double> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::string error;

    EXPECT_TRUE(fast_lio::validate_extrinsics(translation, rotation, &error));
    EXPECT_TRUE(error.empty());
}

TEST(ExtrinsicValidation, RejectsWrongArrayLengths)
{
    const std::vector<double> translation;
    const std::vector<double> rotation(9U, 0.0);
    std::string error;

    EXPECT_FALSE(fast_lio::validate_extrinsics(translation, rotation, &error));
    EXPECT_NE(error.find("exactly 3"), std::string::npos);

    const std::vector<double> valid_translation(3U, 0.0);
    const std::vector<double> short_rotation(8U, 0.0);
    error.clear();
    EXPECT_FALSE(fast_lio::validate_extrinsics(valid_translation, short_rotation, &error));
    EXPECT_NE(error.find("exactly 9"), std::string::npos);
}

TEST(ExtrinsicValidation, RejectsNonFiniteValues)
{
    const std::vector<double> translation{
        0.0, std::numeric_limits<double>::quiet_NaN(), 0.0};
    const std::vector<double> rotation(9U, 0.0);
    std::string error;

    EXPECT_FALSE(fast_lio::validate_extrinsics(translation, rotation, &error));
    EXPECT_NE(error.find("non-finite"), std::string::npos);

    const std::vector<double> valid_translation(3U, 0.0);
    const std::vector<double> infinite_rotation{
        1.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(), 0.0,
        0.0, 0.0, 1.0};
    error.clear();
    EXPECT_FALSE(fast_lio::validate_extrinsics(valid_translation, infinite_rotation, &error));
    EXPECT_NE(error.find("non-finite"), std::string::npos);
}
