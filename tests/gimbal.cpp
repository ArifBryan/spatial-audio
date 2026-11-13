// Looking straight down is a singularity of yaw/pitch/roll angles (yaw and roll swing by 180 degrees together).
// The head pose must be smoothed as a rotation so the virtual speakers do not jump there.
#define main head_track_main
#include "../src/head-track.cpp"
#undef main

// Exact inverse of headMatrix = Rx(r) Ry(p) Rz(y), the way a tracker reports angles (roll forced to 0 at the pole).
static void toEuler(const Mat& M, double& y, double& p, double& r) {
    double sp = std::max(-1.0, std::min(1.0, M.m[0][2])); p = deg(std::asin(sp));
    if (std::fabs(sp) < 0.999999) { y = deg(std::atan2(-M.m[0][1], M.m[0][0])); r = deg(std::atan2(-M.m[1][2], M.m[2][2])); }
    else { r = 0; y = deg(std::atan2(M.m[1][0], M.m[1][1])); }
}
static double angleBetween(Vec a, Vec b) { double d = a.x * b.x + a.y * b.y + a.z * b.z; return deg(std::acos(std::max(-1.0, std::min(1.0, d)))); }
static Vec apply(const Mat& M, Vec v) { return {M.m[0][0] * v.x + M.m[0][1] * v.y + M.m[0][2] * v.z, M.m[1][0] * v.x + M.m[1][1] * v.y + M.m[1][2] * v.z, M.m[2][0] * v.x + M.m[2][1] * v.y + M.m[2][2] * v.z}; }

int main() {
    // The head turns right while nodding forward past the vertical (pitch 0 -> -130 over 4 s at 25 packets/s).
    const int N = 100; Quat qs; bool have = false; Vec prev{}, prevTrue{}; double worstStep = 0, worstErr = 0, trueStep = 0;
    for (int i = 0; i <= N; ++i) {
        double t = i / (double)N; Mat Mt = headMatrix(90 * t, -130 * t, 0); double y, p, r; toEuler(Mt, y, p, r);
        Quat qc = Quat::fromMat(headMatrix(y, p, r)); qs = have ? slerp(qs, qc, 0.35) : qc; have = true;
        Vec v = qs.rotate(speakerDir(0)), truth = apply(Mt, speakerDir(0));
        if (i) { worstStep = std::max(worstStep, angleBetween(v, prev)); trueStep = std::max(trueStep, angleBetween(truth, prevTrue)); }
        worstErr = std::max(worstErr, angleBetween(v, truth)); prev = v; prevTrue = truth;
    }
    bool ok = worstStep < trueStep * 1.5 + 0.5 && worstErr < 4.0;
    printf("looking down past vertical: %s  largest step %.1f deg (real head: %.1f), largest position error %.1f deg\n", ok ? "ok  " : "FAIL", worstStep, trueStep, worstErr);
    // Reduced strength: the speakers must not jump when the head passes 180 degrees (backwards), at any strength.
    bool ok2 = true; double worst2 = 0;
    for (double share : {1.0, 0.7, 0.5, 0.2, 0.0}) {
        Vec pv{}; double w = 0;
        for (double yaw = 100; yaw <= 260; yaw += 0.5) {
            Quat q = scaleRotation(Quat::fromMat(headMatrix(wrap(yaw), 0, 0)), share); Vec v = q.rotate(speakerDir(0));
            if (yaw > 100) w = std::max(w, angleBetween(v, pv)); pv = v;
        }
        worst2 = std::max(worst2, w); if (w > 3.0) ok2 = false;   // the stage speeds up towards the back to catch up; a seam would be 100+ degrees
    }
    printf("rotation scaling through 180 degrees: %s  largest step %.2f deg per 0.5 deg of head turn\n", ok2 ? "ok  " : "FAIL", worst2);
    return ok && ok2 ? 0 : 1;
}
