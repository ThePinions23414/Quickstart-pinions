package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.PinionLib.DriveToPose;
import org.firstinspires.ftc.teamcode.PinionLib.Drivetrain;

@TeleOp
public class starterbotdrive extends OpMode {

    Drivetrain mDrive;
    DcMotor intake;
    CRServo leftWheel;
    CRServo rightWheel;
    double driveSpeed = 0.75;

    DriveToPose mDriveToPose;


    @Override
    public void init() {

        mDrive = new Drivetrain(hardwareMap.dcMotor.get("frontLeft"),
                                hardwareMap.dcMotor.get("backLeft"),
                                hardwareMap.dcMotor.get("frontRight"),
                                hardwareMap.dcMotor.get("backRight"));
        mDrive.setDriveSpeed(0.5);

        intake = hardwareMap.dcMotor.get("intake");
        leftWheel = hardwareMap.crservo.get("lWheel");
        rightWheel = hardwareMap.crservo.get("rWheel");
    }

    @Override
    public void loop() {

//        if(gamepad1.a) {
//            mDriveToPose.driveToPose(current, target, mDrive);
//        }
//        else if(gamepad1.b) {
//            mDriveToPose.driveToPose(current, target2, mDrive);
//        }
//        else {
//            mDrive.gamepadDrive(gamepad1);
//        }
        mDrive.gamepadDrive(gamepad1);

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
