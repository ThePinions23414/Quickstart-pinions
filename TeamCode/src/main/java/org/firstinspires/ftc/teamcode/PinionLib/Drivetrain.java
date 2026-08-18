package org.firstinspires.ftc.teamcode.PinionLib;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drivetrain {
    DcMotor fL;
    DcMotor bL;
    DcMotor fR;
    DcMotor bR;
    double driveSpeed = 1;

    public Drivetrain(DcMotor fLp, DcMotor bLp, DcMotor fRp, DcMotor bRp) {
        fL = fLp;
        bL = bLp;
        fR = fRp;
        bR = bRp;

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDriveSpeed(double speed){
        driveSpeed = speed;
    }

    public void gamepadDrive(Gamepad driveController){
        double y = -driveController.left_stick_y;
        double x = driveController.left_stick_x;
        double rx = driveController.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator * driveSpeed);
        double backLeftPower = ((y - x + rx) / denominator * driveSpeed);
        double frontRightPower = ((y - x - rx) / denominator * driveSpeed);
        double backRightPower = ((y + x - rx) / denominator * driveSpeed);

        fL.setPower(frontLeftPower);
        bL.setPower(backLeftPower);
        fR.setPower(frontRightPower);
        bR.setPower(backRightPower);
    }

    public void drive(double vx, double vy){
        double y = -vy;
        double x = vx;
        double rx = 0;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator * driveSpeed);
        double backLeftPower = ((y - x + rx) / denominator * driveSpeed);
        double frontRightPower = ((y - x - rx) / denominator * driveSpeed);
        double backRightPower = ((y + x - rx) / denominator * driveSpeed);

        fL.setPower(frontLeftPower);
        bL.setPower(backLeftPower);
        fR.setPower(frontRightPower);
        bR.setPower(backRightPower);
    }

    public void turn(double vr){
        double y = -0;
        double x = 0;
        double rx = vr;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator * driveSpeed);
        double backLeftPower = ((y - x + rx) / denominator * driveSpeed);
        double frontRightPower = ((y - x - rx) / denominator * driveSpeed);
        double backRightPower = ((y + x - rx) / denominator * driveSpeed);

        fL.setPower(frontLeftPower);
        bL.setPower(backLeftPower);
        fR.setPower(frontRightPower);
        bR.setPower(backRightPower);
    }

    public void driveAndTurn(double vx, double vy, double vr){
        double y = -vy;
        double x = vx;
        double rx = vr;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator * driveSpeed);
        double backLeftPower = ((y - x + rx) / denominator * driveSpeed);
        double frontRightPower = ((y - x - rx) / denominator * driveSpeed);
        double backRightPower = ((y + x - rx) / denominator * driveSpeed);

        fL.setPower(frontLeftPower);
        bL.setPower(backLeftPower);
        fR.setPower(frontRightPower);
        bR.setPower(backRightPower);
    }

    public void driveAndTurnFieldRelative(double vx, double vy, double vr, double headingRad){
        // Rotate the movement direction counter to the bot's rotation
        double rotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
        double rotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(vr), 1);
        double frontLeftPower = (rotY + rotX + vr) / denominator;
        double backLeftPower = (rotY - rotX + vr) / denominator;
        double frontRightPower = (rotY - rotX - vr) / denominator;
        double backRightPower = (rotY + rotX - vr) / denominator;

        fL.setPower(frontLeftPower);
        bL.setPower(backLeftPower);
        fR.setPower(frontRightPower);
        bR.setPower(backRightPower);
    }
}
