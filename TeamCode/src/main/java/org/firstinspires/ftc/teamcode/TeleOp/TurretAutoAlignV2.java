package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.apriltag.AprilTagDetection;

import java.util.List;


@TeleOp
public class TurretAutoAlignV2 extends OpMode {

    // Import private limelight class
    LimeLightClass limeLightClass = new LimeLightClass();

    private TurretMechanismV2  turret = new TurretMechanismV2();

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    @Override
    public void init() {
        // limelight.init(hardwareMap, telemetry);
        turret.init(hardwareMap);
        limeLightClass.init(hardwareMap);
        limeLightClass.startLimeLight();


        telemetry.addLine("Initialized all mechanisms");
    }

    @Override
    public void start() {
        turret.resetTimer();

    }

    @Override
    public void loop() {
        // run vision logic - ll.update, he did "aprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);" to only get id 20

        LLResult id20 = limeLightClass.getSpecificId(20);
        turret.update(id20);
        limeLightClass.update();


        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }

        if (gamepad1.dpadRightWasPressed()){
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()){
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadDownWasPressed()){
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }

        if (id20 != null){
            telemetry.addLine("I saw the sign");
        }



        telemetry.addData("Tuning P","%.5f (D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D","%.5f (D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size","%.5f (B Button)", stepSizes[stepIndex]);
    }
}
