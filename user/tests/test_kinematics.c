#include "swerve_kinematics.h"
#include "inspection_config.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define RAD(degrees) ((degrees) * (PI / 180.0f))

static const SwerveGeometry geometry = {
    INSPECTION_WHEEL_RADIUS_M, INSPECTION_PIVOT_TRACK_M, INSPECTION_PIVOT_WHEELBASE_M
};
static const float zero[SWERVE_WHEELS] = {0};

static void Near(float a, float b) { assert(fabsf(a - b) < 0.00002f); }

/* Independent rigid-body check in Cartesian coordinates, including signed
   wheel speeds after optimization. Literal pivot positions catch config/order
   mistakes (in particular, using the old 282.23mm tire-center distance). */
static void CheckVectors(const SwerveTargets *w, SwerveTwist t, float scale)
{
    const float x[4] = {0.1975f,0.1975f,-0.1975f,-0.1975f};
    const float y[4] = {0.1425f,-0.1425f,0.1425f,-0.1425f};
    for (unsigned i = 0; i < 4; ++i) {
        float speed = w->wheel[i].speed_rad_s * 0.0625f;
        Near(speed * cosf(w->wheel[i].angle_rad), (t.vx_m_s - t.wz_rad_s * y[i]) * scale);
        Near(speed * sinf(w->wheel[i].angle_rad), (t.vy_m_s + t.wz_rad_s * x[i]) * scale);
    }
}

static void TestInverse(void)
{
    SwerveTargets result;
    SwerveTwist t = {1,0,0};
    assert(Swerve_Inverse(&geometry, &t, zero, &result));
    for (unsigned i = 0; i < 4; ++i) {
        Near(result.wheel[i].speed_rad_s, 16); Near(result.wheel[i].angle_rad, 0);
    }
    t = (SwerveTwist){0,1,0};
    assert(Swerve_Inverse(&geometry, &t, zero, &result));
    for (unsigned i = 0; i < 4; ++i) {
        Near(result.wheel[i].speed_rad_s, 16); Near(result.wheel[i].angle_rad, PI/2);
    }
    t = (SwerveTwist){0,0,1};
    assert(Swerve_Inverse(&geometry, &t, zero, &result)); CheckVectors(&result, t, 1);
    assert(result.wheel[0].angle_rad > PI/2 && result.wheel[1].angle_rad > 0);
    assert(result.wheel[2].angle_rad < -PI/2 && result.wheel[3].angle_rad < 0);
    for (unsigned i = 0; i < 4; ++i) Near(result.wheel[i].speed_rad_s, result.wheel[0].speed_rad_s);

    float reference[4] = {0.1f,-0.2f,0.3f,-0.4f};
    t = (SwerveTwist){0,0,0};
    assert(Swerve_Inverse(&geometry, &t, reference, &result));
    for (unsigned i = 0; i < 4; ++i) {
        assert(result.wheel[i].speed_rad_s == 0); Near(result.wheel[i].angle_rad, reference[i]);
    }
    /* The instantaneous center of rotation lies exactly at the FL pivot. */
    t = (SwerveTwist){0.1425f,-0.1975f,1};
    assert(Swerve_Inverse(&geometry, &t, reference, &result));
    assert(result.wheel[0].speed_rad_s == 0); Near(result.wheel[0].angle_rad, reference[0]);
    CheckVectors(&result, t, 1);
}

static void TestSteering(void)
{
    SwerveSteeringLimit limits[4];
    float reference[4]; SwerveTargets input, output;
    /* Synthetic limits below are TEST fixtures, not vehicle calibration. */
    for (unsigned i = 0; i < 4; ++i) {
        limits[i] = (SwerveSteeringLimit){RAD(-90),RAD(90)};
        reference[i] = RAD(-10);
        input.wheel[i] = (SwerveWheel){8,RAD(170)};
    }
    assert(Swerve_Optimize(&input, limits, reference, &output));
    for (unsigned i = 0; i < 4; ++i) {
        Near(output.wheel[i].speed_rad_s, -8); Near(output.wheel[i].angle_rad, RAD(-10));
    }
    /* Wrap across +/-pi using the continuous mechanical angle interval. */
    for (unsigned i = 0; i < 4; ++i) {
        limits[i] = (SwerveSteeringLimit){0,RAD(270)};
        reference[i] = RAD(179); input.wheel[i] = (SwerveWheel){8,RAD(-179)};
    }
    assert(Swerve_Optimize(&input, limits, reference, &input)); /* Aliased I/O. */
    for (unsigned i = 0; i < 4; ++i) {
        Near(input.wheel[i].angle_rad, RAD(181)); Near(input.wheel[i].speed_rad_s, 8);
    }
    for (unsigned i = 0; i < 4; ++i) {
        limits[i] = (SwerveSteeringLimit){RAD(-10),RAD(10)};
        reference[i] = 0; input.wheel[i] = (SwerveWheel){0,RAD(150)};
    }
    assert(Swerve_Optimize(&input, limits, reference, &output));
    for (unsigned i = 0; i < 4; ++i) Near(output.wheel[i].angle_rad, 0);
    SwerveTargets before = output;
    input.wheel[3] = (SwerveWheel){8,PI/2}; /* No feasible equivalent. */
    assert(!Swerve_Optimize(&input, limits, reference, &output));
    assert(memcmp(&output, &before, sizeof(output)) == 0);
    input.wheel[3].speed_rad_s = 0; reference[3] = RAD(11);
    assert(!Swerve_Optimize(&input, limits, reference, &output));
}

static void TestMixedAndLimits(void)
{
    SwerveSteeringLimit limits[4];
    const float maximum[4] = {8,7,5,20};
    float reference[4] = {0};
    for (unsigned i = 0; i < 4; ++i) limits[i] = (SwerveSteeringLimit){RAD(-135),RAD(135)};
    for (int a = -6; a <= 6; ++a) {
        for (int b = -6; b <= 6; ++b) {
            SwerveTwist t = {a * 0.3f,b * 0.2f,(a-b) * 0.4f};
            SwerveTargets raw, optimized, limited;
            assert(Swerve_Inverse(&geometry, &t, reference, &raw)); CheckVectors(&raw, t, 1);
            assert(Swerve_Optimize(&raw, limits, reference, &optimized)); CheckVectors(&optimized, t, 1);
            float scale;
            assert(Swerve_LimitSpeeds(&optimized, maximum, &limited, &scale));
            assert(scale >= 0 && scale <= 1); CheckVectors(&limited, t, scale);
            for (unsigned i = 0; i < 4; ++i) {
                assert(fabsf(limited.wheel[i].speed_rad_s) <= maximum[i] + 0.00001f);
                assert(optimized.wheel[i].angle_rad >= limits[i].min_rad);
                assert(optimized.wheel[i].angle_rad <= limits[i].max_rad);
                Near(limited.wheel[i].angle_rad, optimized.wheel[i].angle_rad);
                reference[i] = optimized.wheel[i].angle_rad;
            }
        }
    }
    SwerveTargets input = {.wheel = {{16,0},{-8,1},{4,-1},{-2,0.5f}}};
    float scale;
    assert(Swerve_LimitSpeeds(&input, maximum, &input, &scale)); Near(scale, 0.5f);
    const float expected[4] = {8,-4,2,-1};
    for (unsigned i = 0; i < 4; ++i) Near(input.wheel[i].speed_rad_s, expected[i]);
    assert(Swerve_LimitSpeeds(&input, maximum, &input, &scale)); Near(scale, 1);
}

static void TestInvalid(void)
{
    SwerveTargets output = {.wheel = {{1,2},{3,4},{5,6},{7,8}}}, before = output;
    SwerveTwist t = {NAN,0,0};
    assert(!Swerve_Inverse(&geometry, &t, zero, &output));
    t.vx_m_s = INFINITY; assert(!Swerve_Inverse(&geometry, &t, zero, &output));
    t = (SwerveTwist){FLT_MAX,FLT_MAX,FLT_MAX};
    assert(!Swerve_Inverse(&geometry, &t, zero, &output));
    t = (SwerveTwist){1,0,0}; SwerveGeometry bad = geometry;
    bad.wheel_radius_m = 0; assert(!Swerve_Inverse(&bad, &t, zero, &output));
    bad = geometry; bad.pivot_track_m = -1; assert(!Swerve_Inverse(&bad, &t, zero, &output));
    bad = geometry; bad.pivot_wheelbase_m = INFINITY;
    assert(!Swerve_Inverse(&bad, &t, zero, &output));
    float reference[4] = {0,0,0,NAN};
    assert(!Swerve_Inverse(&geometry, &t, reference, &output));
    assert(!Swerve_Inverse(NULL, &t, zero, &output));
    float maximum[4] = {1,1,1,0}, scale = -1;
    assert(!Swerve_LimitSpeeds(&before, maximum, &output, &scale)); assert(scale == -1);
    maximum[3] = INFINITY; assert(!Swerve_LimitSpeeds(&before, maximum, &output, NULL));
    assert(memcmp(&output, &before, sizeof(output)) == 0);
}

void TestKinematics(void)
{
    TestInverse(); TestSteering(); TestMixedAndLimits(); TestInvalid();
    puts("PASS: swerve straight/lateral/yaw/mixed, zero hold, steering reversal/limits, uniform speed saturation, invalid inputs");
}
