package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class LimelightPositioning extends OpMode {
    private Limelight3A limelight;
    double limelightXPos;
    double limelightYPos;
    double limelightAngle;
    double pedroPathingXPos;
    double pedroPathingYPos;
    double limePathingAngle;
    double pedroPathingAngle;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            if (botPose != null) {
                limelightXPos = botPose.getPosition().x;
                limelightYPos = botPose.getPosition().y;
                limelightAngle = botPose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("limelightXPos", limelightXPos);
                telemetry.addData("limelightYPos", limelightYPos);
                telemetry.addData("limelightAngle", limelightAngle);
            }
        }

        pedroPathingXPos = (limelightXPos * 39.3700787402) + 72;
        pedroPathingYPos = (limelightYPos * 39.3700787402) + 72;
        limePathingAngle = limelightAngle + 270;
        if (limePathingAngle >= 360){
            pedroPathingAngle = limePathingAngle - 360;
        } else {
            pedroPathingAngle = limePathingAngle;
        }
        telemetry.addData("pedroPathingXPos", pedroPathingXPos);
        telemetry.addData("pedroPathingYPos", pedroPathingYPos);
        telemetry.addData("pedroPathingAngle", pedroPathingAngle);
    }
}
