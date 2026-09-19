package org.firstinspires.ftc.teamcode.zerex;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ZerexTeleFC extends LinearOpMode {

    public static final double SLOW_SPEED = 0.5;
    public static final double SPEED = 1.0;
    public double speed = 1.0;

    private boolean lastOptionsState = false;
    @Override
    public void runOpMode() throws InterruptedException {
// Declare our motors
// Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
// Reverse the right side motors. This may be wrong for your setup.
// If your robot moves backwards when commanded to go forwards,
// reverse the left side instead.
// See the note about this earlier on this page.
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

// Retrieve the IMU from the hardware map
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
// Adjust the orientation parameters to match your robot



        pinpoint.setOffsets(10, -169, DistanceUnit.MM);

        pinpoint.resetPosAndIMU();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            pinpoint.update();

            if (gamepad1.left_trigger_pressed) {
                speed = SLOW_SPEED;
            } else {
                speed = SPEED;
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

// This button choice was made so that it is hard to hit on accident,
// it can be freely changed based on preference.
// The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options && !lastOptionsState) {
                pinpoint.recalibrateIMU();
            }
            lastOptionsState = gamepad1.options;

            double angleY = pinpoint.getHeading(AngleUnit.RADIANS);
            double xSpeed = Math.copySign(Math.pow(Math.abs(x * Math.cos(-angleY) - y * Math.sin(-angleY)), 2), x * Math.cos(-angleY) - y * Math.sin(-angleY));
            double ySpeed = Math.copySign(Math.pow(Math.abs(x * Math.sin(-angleY) + y * Math.cos(-angleY)), 2), x * Math.sin(-angleY) + y * Math.cos(-angleY));
            double frontLeft = ySpeed + xSpeed + turn;
            double frontRight = ySpeed - xSpeed - turn;
            double backLeft = ySpeed - xSpeed + turn;
            double backRight = ySpeed + xSpeed - turn;
// Denominator is the largest motor power (absolute value) or 1
// This ensures all the powers maintain the same ratio,
// but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(frontLeft), Math.max(Math.abs(frontRight),
                    Math.max(Math.abs(backLeft), Math.abs(backRight))));
            if (denominator > 1.0) {
                frontLeft /= denominator;
                frontRight /= denominator;
                backLeft /= denominator;
                backRight /= denominator;
            }
            frontLeftMotor.setPower(frontLeft*speed);
            backLeftMotor.setPower(backLeft*speed);
            frontRightMotor.setPower(frontRight*speed);
            backRightMotor.setPower(backRight*speed);

        }
    }
}