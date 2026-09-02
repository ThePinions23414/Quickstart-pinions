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
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        fL = hardwareMap.get(DcMotor.class, "frontLeft");
        fR = hardwareMap.get(DcMotor.class, "frontRight");
        bL = hardwareMap.get(DcMotor.class, "backLeft");
        bR = hardwareMap.get(DcMotor.class, "backRight");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

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

        if (llResult.getTx() > 3) {
            fL.setPower(0.25);
            fR.setPower(-0.25);
            bL.setPower(0.25);
            bR.setPower(-0.25);
        } else if (llResult.getTx() < -3) {
            fL.setPower(-0.25);
            fR.setPower(0.25);
            bL.setPower(-0.25);
            bR.setPower(0.25);
        } else {
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
        }
    }
}
