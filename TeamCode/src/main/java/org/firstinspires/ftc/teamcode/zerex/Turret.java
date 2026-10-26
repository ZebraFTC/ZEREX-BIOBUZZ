package org.firstinspires.ftc.teamcode.zerex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Turret {
    DcMotor TurnTable;
    Limelight3A LimeLight;

    public static final int LOWER_LIMIT = -1000;
    public static final int UPPER_LIMIT = 1000;

    public static final double TARGET_DEADBAND_DEGREES = 1.5;

    private final double kP = 0.035;
    private final double kD = 0.004;
    private final ElapsedTime timer;
    private double lastError = 0.0;
    private boolean lastTurnedRight = true;
    private boolean firstFrameOfTargeting = true;

    public Turret(DcMotor TurnTable, Limelight3A LimeLight) {
        this.TurnTable = TurnTable;
        this.LimeLight = LimeLight;
        this.timer = new ElapsedTime();

        TurnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.timer.reset();
    }
    public void setSafePower(double power) {
        int currentPosition = TurnTable.getCurrentPosition();

        if (currentPosition >= UPPER_LIMIT && power > 0) {
            TurnTable.setPower(0.0);
        }
        else if (currentPosition <= LOWER_LIMIT && power < 0) {
            TurnTable.setPower(0.0);
        }
        else {
            TurnTable.setPower(power);
        }
    }
    public void manualControl(double power) {
        firstFrameOfTargeting = true;

        if (Math.abs(power) > 0.05) {
            lastTurnedRight = (power > 0);
            setSafePower(power);
        } else {
            stop();
        }
    }
    public void target() {
        LLResult result = LimeLight.getLatestResult();
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0.0) dt = 0.001;

        if (result != null && result.isValid()) {
            double currentError = result.getTx();
            if (firstFrameOfTargeting) {
                lastError = currentError;
                firstFrameOfTargeting = false;
            }
            if (Math.abs(currentError) <= TARGET_DEADBAND_DEGREES) {
                stop();
                lastError = currentError;
                return;
            }

            lastTurnedRight = (currentError > 0);
            double pTerm = currentError * kP;
            double errorDerivative = (currentError - lastError) / dt;
            double dTerm = errorDerivative * kD;

            double power = pTerm + dTerm;
            power = Math.max(-0.5, Math.min(0.5, power));

            setSafePower(power);
            lastError = currentError;
        } else {
            double searchPower = lastTurnedRight ? 0.15 : -0.15;
            setSafePower(searchPower);
            firstFrameOfTargeting = true;
        }
    }

    public void stop() {
        TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurnTable.setPower(0.0);
    }
}
