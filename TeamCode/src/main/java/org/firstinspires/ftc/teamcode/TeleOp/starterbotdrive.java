package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class starterbotdrive extends OpMode {

    DcMotor fL;
    DcMotor bL;
    DcMotor fR;
    DcMotor bR;
    DcMotor intake;
    CRServo leftWheel;
    CRServo rightWheel;
    double driveSpeed = 0.75;



    @Override
    public void init() {

        fL = hardwareMap.dcMotor.get("frontLeft");
        bL = hardwareMap.dcMotor.get("backLeft");
        fR = hardwareMap.dcMotor.get("frontRight");
        bR = hardwareMap.dcMotor.get("backRight");
        intake = hardwareMap.dcMotor.get("intake");
        leftWheel = hardwareMap.crservo.get("lWheel");
        rightWheel = hardwareMap.crservo.get("rWheel");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

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

        if (gamepad2.aWasPressed()) {
            intake.setPower(1);
            rightWheel.setPower(1);
            leftWheel.setPower(1);
        } else if (gamepad2.aWasReleased()) {
            intake.setPower(0);
            rightWheel.setPower(0);
            leftWheel.setPower(0);
        }

    }

}
