package org.firstinspires.ftc.teamcode.zerex;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret {
    DcMotor TurnTable;
    public static final int LOWER_LIMIT = -1000;
    public static final int UPPER_LIMIT = 1000;

    public Turret(DcMotor TurnTable){
        this.TurnTable = TurnTable;
        TurnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turn(boolean right, double power){
        TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currentPosition = TurnTable.getCurrentPosition();

        if (right == true) {
            if (currentPosition >= UPPER_LIMIT) {
                TurnTable.setPower(0.0);
            } else {
                TurnTable.setPower(power);
            }
        }
        else if (right == false) {
            if (currentPosition <= LOWER_LIMIT) {
                TurnTable.setPower(0.0);
            } else {
                TurnTable.setPower(-power);
            }
        }
    }

    public void target(){

    }

    public void stop() {
        TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurnTable.setPower(0.0);
    }
}
