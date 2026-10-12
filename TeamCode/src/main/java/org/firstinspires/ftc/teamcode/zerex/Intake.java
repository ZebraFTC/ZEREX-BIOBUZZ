package org.firstinspires.ftc.teamcode.zerex;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    DcMotor Intake;
    public Intake(DcMotor Intake){
        this.Intake=Intake;
    }
    public void intake(){
        Intake.setPower(-1.0);
    }
    public void stop(){
        Intake.setPower(0.0);
    }
}
