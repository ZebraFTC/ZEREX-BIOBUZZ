package org.firstinspires.ftc.teamcode.zerex;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ZerexTeleFC extends LinearOpMode {

    public static final double SLOW_SPEED = 0.5;
    public static final double SPEED = 1.0;
    public double speed = 1.0;

    // --- ADRC Drivetrain Constants ---
    // Max velocity of your motor (e.g., standard goBILDA Yellow Jacket 19.2:1 runs at ~2800 ticks/sec max loop velocity)
    public static final double MAX_MOTOR_TICKS_PER_SEC = 2500.0;
    public static final double b0 = 3000.0;
    public static final double omega_c = 12.0;
    public static final double omega_o = 48.0;

    // Allocate 4 individual controllers (one node for each structural motor layout)
    private ControlAlgorithm.AdrcController adrcFL;
    private ControlAlgorithm.AdrcController adrcBL;
    private ControlAlgorithm.AdrcController adrcFR;
    private ControlAlgorithm.AdrcController adrcBR;

    private ElapsedTime runtime = new ElapsedTime();

    // Feature flag toggles
    private boolean useAdrc = false;
    private boolean lastXState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declared explicit DcMotorEx variables to capture high precision velocity feedback signals
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        // Clear out internal built-in profile layers to delegate full authority to your ADRC engines
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Instantiate separate independent math tracking loops for each corner
        adrcFL = new ControlAlgorithm.AdrcController(b0, omega_o, omega_c);
        adrcBL = new ControlAlgorithm.AdrcController(b0, omega_o, omega_c);
        adrcFR = new ControlAlgorithm.AdrcController(b0, omega_o, omega_c);
        adrcBR = new ControlAlgorithm.AdrcController(b0, omega_o, omega_c);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(10, -169, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();

        telemetry.addData("System Ready", "Press Play to Start. Toggle ADRC using Gamepad1 X.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();
        adrcFL.reset();
        adrcBL.reset();
        adrcFR.reset();
        adrcBR.reset();

        while (opModeIsActive()) {
            pinpoint.update();
            double currentTime = runtime.seconds();

            // --- Adaptive Velocity Driver Precision Scaling ---
            if (gamepad1.left_trigger > 0.3) {
                speed = SLOW_SPEED;
            } else {
                speed = SPEED;
            }

            // --- State Toggle Interface Edge-Detection (Gamepad 1 Button X) ---
            boolean currentXState = gamepad1.x;
            if (currentXState && !lastXState) {
                useAdrc = !useAdrc; // Reverse Boolean flag setting

                // Clear integration memory bounds to neutralize potential starting jump shocks
                adrcFL.reset();
                adrcBL.reset();
                adrcFR.reset();
                adrcBR.reset();
            }
            lastXState = currentXState;

            // --- Capture Joystick Target Inputs ---
            double y = -gamepad1.left_stick_y * speed;
            double x = gamepad1.left_stick_x * speed;
            double rx = gamepad1.right_stick_x * speed;

            // --- Field-Centric Coordinate Rotation ---
            double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Mecanum inverse kinematics denominator scaling factor
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            // Generate standard target percentages ranging across -1.0 to 1.0 thresholds
            double targetFLPower = (rotY + rotX + rx) / denominator;
            double targetBLPower = (rotY - rotX + rx) / denominator;
            double targetFRPower = (rotY - rotX - rx) / denominator;
            double targetBRPower = (rotY + rotX - rx) / denominator;

            double finalFL, finalBL, finalFR, finalBR;

            if (useAdrc) {
                // Translate raw power scaling percentages directly into explicit Target Encoder Ticks/Second Velocities
                double targetVelFL = targetFLPower * MAX_MOTOR_TICKS_PER_SEC;
                double targetVelBL = targetBLPower * MAX_MOTOR_TICKS_PER_SEC;
                double targetVelFR = targetFRPower * MAX_MOTOR_TICKS_PER_SEC;
                double targetVelBR = targetBRPower * MAX_MOTOR_TICKS_PER_SEC;

                // Capture real-time velocity parameters from motor encoder interfaces
                double currentVelFL = frontLeftMotor.getVelocity();
                double currentVelBL = backLeftMotor.getVelocity();
                double currentVelFR = frontRightMotor.getVelocity();
                double currentVelBR = backRightMotor.getVelocity();

                // Compute stabilized output commands via the active error cancellation engines
                finalFL = adrcFL.update(targetVelFL, currentVelFL, currentTime);
                finalBL = adrcBL.update(targetVelBL, currentVelBL, currentTime);
                finalFR = adrcFR.update(targetVelFR, currentVelFR, currentTime);
                finalBR = adrcBR.update(targetVelBR, currentVelBR, currentTime);
            } else {
                //  if flag tracker is manually toggled off, then Pass direct voltage percentages seamlessly
                finalFL = targetFLPower;
                finalBL = targetBLPower;
                finalFR = targetFRPower;
                finalBR = targetBRPower;
            }

            // Push voltage commands down out to the actuators
            frontLeftMotor.setPower(finalFL);
            backLeftMotor.setPower(finalBL);
            frontRightMotor.setPower(finalFR);
            backRightMotor.setPower(finalBR);

            // ---Telemetry Diagnostics Panel Outputs---
            telemetry.addData("ADRC Drive Mode Status", useAdrc ? "ACTIVE (X to disable)" : "DISABLED (X to enable)");
            telemetry.addData("Speed Mode Profile Modifier", speed);
            telemetry.addData("Heading Angle (Rad)", "%.2f", botHeading);

            if (useAdrc) {
                telemetry.addData("FL Target vs Measured Vel", "%.0f / %.0f", targetFLPower * MAX_MOTOR_TICKS_PER_SEC, frontLeftMotor.getVelocity());
                telemetry.addData("FL Dynamic Disturbance Correction (z2)", "%.2f", adrcFL.getEstimatedDisturbance());
            }
            telemetry.update();
        }
    }
}
