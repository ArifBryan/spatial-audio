// Real spherical harmonics up to order 3 (ACN channel order, SN3D normalisation, as in AmbiX).
// Shared by spatial-tools (fits the HRTF into these channels) and the ambirot plugin (encodes sources into them), so the two
// can never disagree. Direction: x forward, y left, z up (unit vector).
#pragma once
#include <cmath>

constexpr int SH_ORDER = 3, SH_CHANNELS = (SH_ORDER + 1) * (SH_ORDER + 1);

inline void shEval(double x, double y, double z, double* Y) {
    const double s3 = std::sqrt(3.0), s5_8 = std::sqrt(5.0 / 8.0), s3_8 = std::sqrt(3.0 / 8.0), s15 = std::sqrt(15.0);
    Y[0] = 1.0;
    Y[1] = y; Y[2] = z; Y[3] = x;
    Y[4] = s3 * x * y; Y[5] = s3 * y * z; Y[6] = 0.5 * (3 * z * z - 1); Y[7] = s3 * x * z; Y[8] = 0.5 * s3 * (x * x - y * y);
    Y[9] = s5_8 * y * (3 * x * x - y * y); Y[10] = s15 * x * y * z; Y[11] = s3_8 * y * (5 * z * z - 1); Y[12] = 0.5 * z * (5 * z * z - 3);
    Y[13] = s3_8 * x * (5 * z * z - 1); Y[14] = 0.5 * s15 * z * (x * x - y * y); Y[15] = s5_8 * x * (x * x - 3 * y * y);
}
