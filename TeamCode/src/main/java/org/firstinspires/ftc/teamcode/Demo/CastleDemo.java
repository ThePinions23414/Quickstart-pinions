package org.firstinspires.ftc.teamcode.Demo;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp
public class CastleDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {





        DcMotor fL = hardwareMap.dcMotor.get("frontLeft");
        DcMotor bL = hardwareMap.dcMotor.get("backLeft");
        DcMotor fR = hardwareMap.dcMotor.get("frontRight");
        DcMotor bR = hardwareMap.dcMotor.get("backRight");
        DcMotor lIntake = hardwareMap.dcMotor.get("leftIntake");
        DcMotor rIntake = hardwareMap.dcMotor.get("rightIntake");
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        DcMotor roller = hardwareMap.dcMotor.get("upperRoller");
        CRServo bLRamp = hardwareMap.crservo.get("leftLowerRamp");
        CRServo bRRamp = hardwareMap.crservo.get("rightLowerRamp");
        CRServo middleRoller = hardwareMap.crservo.get("middleRoller");
        CRServo tLRamp = hardwareMap.crservo.get("leftUpperRamp");
        CRServo tRRamp = hardwareMap.crservo.get("rightUpperRamp");
        Servo gate = hardwareMap.servo.get("gate");
        Servo shootAdjust = hardwareMap.servo.get("shooterAdjuster");

        double driveSpeed = 0.4;
        double Power = 1;


        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();



        if (isStopRequested()) {
            return;


        }


        while (opModeIsActive()) {


            shootAdjust.setPosition(0.395);
            Power = 1;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator * driveSpeed);
            double backLeftPower = ((y - x + rx) / denominator * driveSpeed);
            double frontRightPower = ((y - x - rx) / denominator * driveSpeed);
            double backRightPower = ((y + x - rx) / denominator * driveSpeed);

            fL.setPower(frontLeftPower);
            bL.setPower(backLeftPower);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);

            if (gamepad2.dpad_left){
                lIntake.setPower(1);
                rIntake.setPower(-1);
                gate.setPosition(0.3);
                bLRamp.setPower(1);
                bRRamp.setPower(-1);
                tLRamp.setPower(1);
                tRRamp.setPower(-1);


            }

            if (gamepad2.dpadLeftWasPressed()) {
                middleRoller.setPower(1);
            } else if (gamepad2.dpadLeftWasReleased()) {
                middleRoller.setPower(0);
            }
            lIntake.setPower(0);
            rIntake.setPower(0);
            roller.setPower(0);



            if (gamepad2.dpad_right) {
                bLRamp.setPower(0);
                bRRamp.setPower(0);
                tLRamp.setPower(0);
                tRRamp.setPower(0);
                shooter.setPower(0);
            }
            if (gamepad2.x) {
                shooter.setPower(Power);
            }
            if (gamepad2.b){
                gate.setPosition(0);
                roller.setPower(-1);
            }


            if (gamepad2.dpad_down){
                lIntake.setPower(1);
                rIntake.setPower(-1);
                roller.setPower(0);
                middleRoller.setPower(-1);
            }




            telemetry.update();
        }

    }
}









