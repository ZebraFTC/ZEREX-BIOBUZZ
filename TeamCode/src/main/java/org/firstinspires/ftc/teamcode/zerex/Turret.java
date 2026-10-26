package org.firstinspires.ftc.teamcode.zerex;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Turret {
    DcMotor TurnTable;
    Limelight3A LimeLight;

    public static final int LOWER_LIMIT = -1000;
    public static final int UPPER_LIMIT = 1000;

    // Remembers the last known direction of the AprilTag to prevent freezing if it goes out of frame
    private boolean lastTurnedRight = true;
    public Turret(DcMotor TurnTable, Limelight3A LimeLight) {
        this.TurnTable = TurnTable;
        this.LimeLight = LimeLight;
        TurnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turn(boolean right, double power) {
        TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currentPosition = TurnTable.getCurrentPosition();
        if (right == true) {
            if (currentPosition >= UPPER_LIMIT) {
                TurnTable.setPower(0.0);
            } else {
                TurnTable.setPower(-power);
            }
        } else if (right == false) {
            if (currentPosition <= LOWER_LIMIT) {
                TurnTable.setPower(0.0);
            } else {
                TurnTable.setPower(power);
            }
        }
    }

    public void target() {
        LLResult result = LimeLight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double kP = 0.04;
            double power = tx * kP;

            power = Math.max(-0.5, Math.min(0.5, power));

            if (tx > 1.5) {
                lastTurnedRight = true;
                turn(true, Math.abs(power));
            } else if (tx < -1.5) {
                lastTurnedRight = false;
                turn(false, Math.abs(power));
            } else {
                stop();
            }
        } else {
            turn(lastTurnedRight, 0.15);
        }
    }

    public void stop() {
        TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurnTable.setPower(0.0);
    }
}