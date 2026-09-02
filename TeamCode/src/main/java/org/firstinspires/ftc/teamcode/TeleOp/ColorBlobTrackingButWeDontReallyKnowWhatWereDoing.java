package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp
public class ColorBlobTrackingButWeDontReallyKnowWhatWereDoing extends OpMode {

    private Limelight3A limelight;
//    private DcMotor fL;
//    private DcMotor fR;
//    private DcMotor bL;
//    private DcMotor bR;

    private ColorBlobMechanism  colorBlob = new ColorBlobMechanism();

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};
    int stepIndex = 2;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        colorBlob.init(hardwareMap);

//        fL.setDirection(DcMotorSimple.Direction.REVERSE);
//        bL.setDirection(DcMotorSimple.Direction.REVERSE);

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
        }

//        if (llResult.getTx() > 3) {
//            fL.setPower(0.25);
//            fR.setPower(-0.25);
//            bL.setPower(0.25);
//            bR.setPower(-0.25);
//        } else if (llResult.getTx() < -3) {
//            fL.setPower(-0.25);
//            fR.setPower(0.25);
//            bL.setPower(-0.25);
//            bR.setPower(0.25);
//        } else {
//            fL.setPower(0);
//            fR.setPower(0);
//            bL.setPower(0);
//            bR.setPower(0);
//        }


        colorBlob.update(llResult);

//        colorBlob.setkP(0.0701);
//        colorBlob.setkD(0.00001);


        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            colorBlob.setkP(colorBlob.getkP() - stepSizes[stepIndex]);
        }

        if (gamepad1.dpadRightWasPressed()){
            colorBlob.setkP(colorBlob.getkP() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()){
            colorBlob.setkD(colorBlob.getkD() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadDownWasPressed()){
            colorBlob.setkD(colorBlob.getkD() - stepSizes[stepIndex]);
        }

        if (llResult != null){
            telemetry.addLine("I saw the sign");
        }
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = ((y + x + rx) / denominator * driveSpeed);
//        double backLeftPower = ((y - x + rx) / denominator * driveSpeed);
//        double frontRightPower = ((y - x - rx) / denominator * driveSpeed);
//        double backRightPower = ((y + x - rx) / denominator * driveSpeed);
//
//        fL.setPower(frontLeftPower);
//        bL.setPower(backLeftPower);
//        fR.setPower(frontRightPower);
//        bR.setPower(backRightPower);



        telemetry.addData("Tuning P","%.5f (D-Pad L/R)", colorBlob.getkP());
        telemetry.addData("Tuning D","%.5f (D-Pad U/D)", colorBlob.getkD());
        telemetry.addData("Step Size","%.5f (B Button)", stepSizes[stepIndex]);
    }
}
