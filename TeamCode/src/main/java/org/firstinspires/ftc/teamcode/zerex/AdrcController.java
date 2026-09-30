package org.firstinspires.ftc.teamcode.zerex;

/**
 * First-Order Linear Active Disturbance Rejection Control (LADRC).
 */
public class AdrcController {
    private double b0;
    private double omega_o;
    private double omega_c;

    private double l1, l2;
    private double z1 = 0;
    private double z2 = 0;

    private double lastTime = 0;
    private boolean initialized = false;
    private double lastControlEffort = 0;

    public AdrcController(double b0, double omega_o, double omega_c) {
        this.b0 = b0;
        setTuning(omega_o, omega_c);
    }

    public void setTuning(double omega_o, double omega_c) {
        this.omega_o = omega_o;
        this.omega_c = omega_c;
        this.l1 = 2 * omega_o;
        this.l2 = omega_o * omega_o;
    }

    public void reset() {
        z1 = 0;
        z2 = 0;
        initialized = false;
        lastControlEffort = 0;
    }

    public double update(double reference, double measurement, double currentTimeSeconds) {
        if (!initialized) {
            lastTime = currentTimeSeconds;
            z1 = measurement;
            z2 = 0;
            initialized = true;
            return 0.0;
        }

        double dt = currentTimeSeconds - lastTime;
        if (dt <= 0.0) dt = 0.001;
        lastTime = currentTimeSeconds;

        double error_o = measurement - z1;

        double z1_dot = z2 + b0 * lastControlEffort + l1 * error_o;
        double z2_dot = l2 * error_o;

        z1 += z1_dot * dt;
        z2 += z2_dot * dt;

        double e_c = reference - z1;
        double u0 = omega_c * e_c;

        double u = (u0 - z2) / b0;

        if (u > 1.0) u = 1.0;
        else if (u < -1.0) u = -1.0;

        lastControlEffort = u;
        return u;
    }

    public double getEstimatedState() { return z1; }
    public double getEstimatedDisturbance() { return z2; }
}
