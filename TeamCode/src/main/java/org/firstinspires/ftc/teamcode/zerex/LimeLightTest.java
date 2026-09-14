package org.firstinspires.ftc.teamcode.zerex;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous
public class LimeLightTest extends OpMode {
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    @Override
    public void init() {
        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);//april tag pipeline
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(10,  -169, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
    }
    @Override
    public void start() {
        limelight.start();
    }
    @Override
    public void loop() {
        pinpoint.update();
        double headingDegrees = pinpoint.getHeading(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(headingDegrees);
        LLResult llResult= limelight.getLatestResult();
        if (llResult !=null && llResult.isValid()){
            Pose3D botpose=llResult.getBotpose_MT2();
            telemetry.addData("Tx",llResult.getTx());
            telemetry.addData("Ty",llResult.getTy());
            telemetry.addData("Ta",llResult.getTa());
            telemetry.addData("BotPose",botpose.toString());
            telemetry.addData("Yaw",botpose.getOrientation().getYaw());

        }
    }

}