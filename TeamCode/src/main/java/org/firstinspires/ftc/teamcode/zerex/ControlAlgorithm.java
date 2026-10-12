package org.firstinspires.ftc.teamcode.zerex;

public class ControlAlgorithm {
    public static class AdrcController { //Active Disturbance Rejection Control

        // --- Tuning Variables ---
        public double b0; // Control Gain Parameters
        public double omega_o; // Observer Bandwidth
        public double omega_c; // Controller Bandwidth

        // --- Gains & States ---
        private double l1, l2; //First & Second Observer Gain(ESO)
        private double z1 = 0; // State estimate(Velocity)
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
        private double kp; //Proportional Gain
        private double ki; //Integral Gain
        private double kd; //Derivative Gain

        // --- Math States ---
        private double integral = 0; //Accumulated Error
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
            private double kp; //Proportional Gain
            private double ki; //Integral Gain

            private double integral = 0.0;
            private double lastTime = 0.0;
            private boolean initialized = false;

            public PiController(double kp, double ki) {
                setTuning(kp, ki);
            }

            public void setTuning(double kp, double ki) {
                this.kp = kp;
                this.ki = ki;
            }

            public void reset() {
                integral = 0.0;
                lastTime = 0.0;
                initialized = false;
            }

            public double update(double reference, double measurement, double currentTimeSeconds) {
                double error = reference - measurement;

                if (!initialized) {
                    lastTime = currentTimeSeconds;
                    initialized = true;
                    return 0.0;
                }

                double dt = currentTimeSeconds - lastTime;
                if (dt <= 0.0) dt = 0.001;
                lastTime = currentTimeSeconds;

                integral += error * dt;
                integral = Math.max(-1.0, Math.min(1.0, integral));

                double output = kp * error + ki * integral;

                return Math.max(-1.0, Math.min(1.0, output));
            }

            public double getIntegral() {
                return integral;
            }
        }

        public static class DupidController { //Dynamically Updated Proportional Integral Derivative
            private double kp; //Proportional Gain
            private double ki; //Integral Gain
            private double kd; //Derivative Gain

            private double integral = 0.0;
            private double previousError = 0.0;
            private double lastTime = 0.0;
            private boolean initialized = false;

            private double integralLimit = 1.0;
            private double outputMin = -1.0;
            private double outputMax = 1.0;

            public DupidController(double kp, double ki, double kd) {
                setTuning(kp, ki, kd);
            }

            public void setTuning(double kp, double ki, double kd) {
                this.kp = kp;
                this.ki = ki;
                this.kd = kd;
            }

            public void setIntegralLimit(double limit) {
                integralLimit = Math.abs(limit);
                integral = Math.max(-integralLimit, Math.min(integralLimit, integral));
            }

            public void setOutputLimits(double min, double max) {
                if (min > max) {
                    throw new IllegalArgumentException(
                            "Minimum output must not exceed maximum output.");
                }

                outputMin = min;
                outputMax = max;
            }

            public void reset() {
                integral = 0.0;
                previousError = 0.0;
                lastTime = 0.0;
                initialized = false;
            }

            public double update(double reference, double measurement, double currentTimeSeconds) {
                double error = reference - measurement;

                if (!initialized) {
                    lastTime = currentTimeSeconds;
                    previousError = error;
                    initialized = true;
                    return 0.0;
                }

                double dt = currentTimeSeconds - lastTime;
                if (dt <= 0.0) dt = 0.001;
                lastTime = currentTimeSeconds;

                double derivative = (error - previousError) / dt;

                double candidateIntegral = Math.max(
                        -integralLimit,
                        Math.min(integralLimit, integral + error * dt)
                );

                double candidateOutput =
                        kp * error + ki * candidateIntegral + kd * derivative;

                boolean pushingHigh =
                        candidateOutput > outputMax && error * ki > 0.0;

                boolean pushingLow =
                        candidateOutput < outputMin && error * ki < 0.0;

                if (!pushingHigh && !pushingLow) {
                    integral = candidateIntegral;
                }

                double output = kp * error + ki * integral + kd * derivative;

                previousError = error;

                return Math.max(outputMin, Math.min(outputMax, output));
            }

            public double getIntegral() {
                return integral;
            }

            public double getError() {
                return previousError;
            }
        }

        public static class FfdidController { //Feedforward w/ Derivative
            private double kf; //Feedforward Gain
            private double kp; //Proportional Gain
            private double kd; //Derivative Gain

            private double previousError = 0.0; //
            private double lastTime = 0.0;
            private boolean initialized = false;

            public FfdidController(double kf, double kp, double kd) {
                setTuning(kf, kp, kd);
            }

            public void setTuning(double kf, double kp, double kd) {
                this.kf = kf;
                this.kp = kp;
                this.kd = kd;
            }

            public void reset() {
                previousError = 0.0;
                lastTime = 0.0;
                initialized = false;
            }

            public double update(double reference, double measurement, double currentTimeSeconds) {
                double error = reference - measurement;

                if (!initialized) {
                    lastTime = currentTimeSeconds;
                    previousError = error;
                    initialized = true;

                    return Math.max(-1.0, Math.min(1.0, kf * reference));
                }

                double dt = currentTimeSeconds - lastTime;
                if (dt <= 0.0) dt = 0.001;
                lastTime = currentTimeSeconds;

                double derivative = (error - previousError) / dt;
                previousError = error;

                double output = kf * reference + kp * error + kd * derivative;

                return Math.max(-1.0, Math.min(1.0, output));
            }

            public double getError() {
                return previousError;
            }
        }
    }
}

