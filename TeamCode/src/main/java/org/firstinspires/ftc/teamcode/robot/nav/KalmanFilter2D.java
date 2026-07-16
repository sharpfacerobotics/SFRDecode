package org.firstinspires.ftc.teamcode.robot.nav;

/**
 * The prediction model: a constant-velocity Kalman filter for one tracked point in the field plane.
 *
 * It fuses noisy position measurements (from a detector / distance sensor) with a constant-velocity
 * motion model to produce a SMOOTHED position AND a velocity estimate, then extrapolates where the
 * point will be a short time ahead. That prediction is what lets the AutoPositioner dodge AHEAD of a
 * moving robot instead of reacting late.
 *
 * It is TUNED, not trained — two knobs, no dataset:
 *   - PROCESS_NOISE (q): how much the target may accelerate between frames. Higher = trust the
 *     measurements more, react faster, jitter more.
 *   - MEASUREMENT_NOISE (r): how noisy the sensor is. Higher = smooth harder, lag more.
 *
 * Under a constant-velocity model the x and y axes are independent, so this runs two decoupled 1-D
 * filters (state = [position, velocity]); the math stays 2x2 and exact. No FTC/SDK imports, so it is
 * unit-testable off-robot.
 */
public class KalmanFilter2D {

    /** Acceleration process-noise density (in^2/s^3). Bigger = snappier, noisier. */
    public static double PROCESS_NOISE = 60.0;
    /** Position measurement-noise variance (in^2). Bigger = smoother, laggier. ~ (sensor std)^2. */
    public static double MEASUREMENT_NOISE = 9.0;

    private final Axis x = new Axis();
    private final Axis y = new Axis();
    private boolean initialized = false;

    /** One decoupled constant-velocity axis: state [p, v] with 2x2 covariance P. */
    private static class Axis {
        double p, v;
        double p00, p01, p10, p11;

        void reset(double pos) {
            p = pos; v = 0;
            p00 = 1e3; p01 = 0; p10 = 0; p11 = 1e3;   // start very uncertain
        }

        /** Predict dt seconds ahead: p += v*dt, then P = F P F^T + Q, F = [[1,dt],[0,1]]. */
        void predict(double dt, double q) {
            p += v * dt;
            double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
            double np00 = p00 + dt * (p10 + p01) + dt2 * p11 + q * dt4 / 4.0;
            double np01 = p01 + dt * p11                     + q * dt3 / 2.0;
            double np10 = p10 + dt * p11                     + q * dt3 / 2.0;
            double np11 = p11                                + q * dt2;
            p00 = np00; p01 = np01; p10 = np10; p11 = np11;
        }

        /** Correct with a position measurement z (H = [1,0]), measurement variance r. */
        void update(double z, double r) {
            double innovation = z - p;
            double s = p00 + r;
            double k0 = p00 / s;      // Kalman gain, K = P H^T / S
            double k1 = p10 / s;
            p += k0 * innovation;
            v += k1 * innovation;
            // P = (I - K H) P,  KH = [[k0,0],[k1,0]]
            double np00 = (1 - k0) * p00;
            double np01 = (1 - k0) * p01;
            double np10 = p10 - k1 * p00;
            double np11 = p11 - k1 * p01;
            p00 = np00; p01 = np01; p10 = np10; p11 = np11;
        }
    }

    /** (Re)start the filter at a measured position with zero assumed velocity. */
    public void init(double px, double py) {
        x.reset(px); y.reset(py);
        initialized = true;
    }

    public boolean isInitialized() { return initialized; }

    /** Advance the motion model by dt seconds with no measurement (coast between frames). */
    public void predict(double dt) {
        if (dt <= 0) return;
        x.predict(dt, PROCESS_NOISE);
        y.predict(dt, PROCESS_NOISE);
    }

    /** Fuse a new position measurement taken dt seconds after the last update. */
    public void update(double px, double py, double dt) {
        if (!initialized) { init(px, py); return; }
        predict(dt);
        x.update(px, MEASUREMENT_NOISE);
        y.update(py, MEASUREMENT_NOISE);
    }

    public double getX()  { return x.p; }
    public double getY()  { return y.p; }
    public double getVx() { return x.v; }
    public double getVy() { return y.v; }

    /** Predicted position dt seconds ahead (constant-velocity extrapolation of the filtered state). */
    public double predictedX(double dt) { return x.p + x.v * dt; }
    public double predictedY(double dt) { return y.p + y.v * dt; }
}
