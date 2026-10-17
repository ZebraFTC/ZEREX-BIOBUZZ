package org.firstinspires.ftc.teamcode.zerex;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Outtake {
    DcMotor Shooter;
    public Outtake(DcMotor Shooter){
        this.Shooter=Shooter;
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void shoot(double power){
        Shooter.setPower(power);
    }
    public void stop(){
        Shooter.setPower(0.0);
    }
}
