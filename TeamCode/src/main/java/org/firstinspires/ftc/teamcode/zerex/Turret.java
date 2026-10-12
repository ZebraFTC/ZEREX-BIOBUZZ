package org.firstinspires.ftc.teamcode.zerex;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret {
    DcMotor TurnTable;
    public Turret(DcMotor TurnTable){
        this.TurnTable=TurnTable;
    }
    public void turn(boolean right,double power){
    TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    if (right==true) {
        TurnTable.setPower(power);
    }
    else if(right==false){
        TurnTable.setPower(-power);
    }
    }
    public void target(){

    }
    public void stop() {
        TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurnTable.setPower(0.0);
    }
}
