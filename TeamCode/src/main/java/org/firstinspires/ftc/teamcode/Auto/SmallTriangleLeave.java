package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name = "SmallTriangleLeave")
public class SmallTriangleLeave extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fL = hardwareMap.dcMotor.get("frontLeft");
        DcMotor bL = hardwareMap.dcMotor.get("backLeft");
        DcMotor fR = hardwareMap.dcMotor.get("frontRight");
        DcMotor bR = hardwareMap.dcMotor.get("backRight");
        Servo spindexer = hardwareMap.get(Servo.class, "spindexer");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        spindexer.setPosition(0.15);


        if (isStopRequested()) {
            return;


        }

        if (opModeIsActive()) {
            fL.setPower(.5);
            fR.setPower(.5);
            bL.setPower(.5);
            bR.setPower(.5);
            sleep(250);
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
        }
    }
}
