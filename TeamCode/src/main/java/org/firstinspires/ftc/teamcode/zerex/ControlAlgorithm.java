package org.firstinspires.ftc.teamcode.zerex;

public class ControlAlgorithm {
    public static class AdrcController { //Active Disturbance Rejection Control

        // --- Tuning Variables ---
        private double b0; // Control gain parameters
        private double omega_o; // Observer Bandwidth
        private double omega_c; // Controller Bandwidth

        // --- Gains & States ---
        private double l1, l2;
        private double z1 = 0; // State estimate
        private double z2 = 0; // Disturbance estimate

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

        public double getEstimatedState() {
            return z1;
        }

        public double getEstimatedDisturbance() {
            return z2;
        }
    }

    public static class PidController { //Proportional Integral Derivative

        // --- Tuning Variable ---
        private double kp; //Proportional
        private double ki; //Integral
        private double kd; //Derivative

        // --- Math States ---
        private double integral = 0;
        private double previousError = 0;
        private double lastTime = 0;
        private boolean initialized = false;

        // Constructor to set tuning coefficients
        public PidController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        // Allows you to dynamically re-tune variables during runtime dashboards
        public void setTuning(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        // Erases historic integral error accumulations when switching modes
        public void reset() {
            this.integral = 0;
            this.previousError = 0;
            this.initialized = false;
        }

        public double update(double reference, double measurement, double currentTimeSeconds) {
            // First loop fallback initialization pass
            if (!initialized) {
                lastTime = currentTimeSeconds;
                initialized = true;
                return 0.0;
            }

            // Calculate precise loop timing delta
            double dt = currentTimeSeconds - lastTime;
            if (dt <= 0.0) dt = 0.001; // Protects against divide-by-zero errors
            lastTime = currentTimeSeconds;

            // System error tracking calculation
            double error = reference - measurement;

            // Proportional Integral Accumulation
            integral += error * dt;

            // --- Integral Anti-Windup Clamping ---
            // Limits the total impact of the I-term so it doesn't spin out of control
            if (integral > 1.0) integral = 1.0;
            else if (integral < -1.0) integral = -1.0;

            // Numeric Derivative tracking step
            double derivative = (error - previousError) / dt;
            previousError = error;

            // Calculate the comprehensive output control command (u)
            double u = (kp * error) + (ki * integral) + (kd * derivative);

            // Cap the output command explicitly to legal FTC voltage thresholds (-1.0 to 1.0)
            if (u > 1.0) u = 1.0;
            else if (u < -1.0) u = -1.0;

            return u;
        }

        public static class PiController { //Proportional Integral

        }

        public static class DupidController { //Dynamically Updated Proportional Integral Derivative

        }

        public static class FfdidController { //Feedforward w/ Derivative

        }
    }
}

