package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelLogicNotSortedST {
    private DcMotorEx lShooter;
    private DcMotorEx rShooter;
    private CRServo upperRoller;
    private CRServo lowerRoller;
    private CRServo lLifter;
    private CRServo rLifter;
    private DcMotorEx turret;
    private Servo gate;
    private Servo leftHood;
    private Servo rightHood;
    private Servo spindexer;
    private DcMotor intake;
    private Limelight3A limelight;
    private ElapsedTime stateTimer = new ElapsedTime();
    private  ElapsedTime shootTimer = new ElapsedTime();
    private Servo light1;
    private Servo light2;
    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH;
    }
    private FlywheelState flywheelState;

    private int shotsRemaining = 0;
    private int shotNumber = 1;
    private double flywheelVelocity = 0;
    private double MIN_FLYWHEEL_RPM = 1400;
    private double TARGET_FLYWHEEL_RPM = 1500;
    private double FLYWHEEL_MAX_SPINUP_TIME = 2.75;
    double P = 140;
    double F = 14.04;
    boolean spinningUp = false;


    private String pattern = "";
    double slot1Position = 0.32;
    double turretPosition = 0;
    double lHoodPosition = 0.915;
    double rHoodPosition = 0.085;



    public void init(HardwareMap hwMap) {
        leftHood = hwMap.get(Servo.class, "leftHood");
        rightHood = hwMap.get(Servo.class, "rightHood");
        upperRoller = hwMap.get(CRServo.class, "upperRoller");
        lowerRoller = hwMap.get(CRServo.class, "lowerRoller");
        lLifter = hwMap.get(CRServo.class, "leftLifter");
        rLifter = hwMap.get(CRServo.class, "rightLifter");
        turret = hwMap.get(DcMotorEx.class, "turret");
        gate = hwMap.get(Servo.class, "gate");
        lShooter = hwMap.get(DcMotorEx.class, "leftShooter");
        rShooter = hwMap.get(DcMotorEx.class, "rightShooter");
        spindexer = hwMap.get(Servo.class, "spindexer");
        intake = hwMap.get(DcMotor.class, "intake");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        lShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelState = FlywheelState.IDLE;

        lShooter.setVelocity(0);
        rShooter.setVelocity(0);
        spindexer.setPosition(slot1Position);
        leftHood.setPosition(lHoodPosition);
        rightHood.setPosition(rHoodPosition);



    }




    public void update() {
        switch (flywheelState) {
            case IDLE:
                gate.setPosition(0.7);
                if (spinningUp) {

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                lShooter.setVelocity(TARGET_FLYWHEEL_RPM);
                rShooter.setVelocity(TARGET_FLYWHEEL_RPM);
                gate.setPosition(0.7);
                gate.setPosition(0.7);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setTargetPosition(0);
                turret.setPower(1);

                lowerRoller.setPower(1);
                upperRoller.setPower(0);
                if(shotsRemaining > 0){
                    stateTimer.reset();
                    shootTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }

                break;
            case LAUNCH:
                if (rShooter.getVelocity() > MIN_FLYWHEEL_RPM || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {

                    gate.setPosition(0.2);

                    lowerRoller.setPower(1);
                    lLifter.setPower(1);
                    rLifter.setPower(-1);
                    intake.setPower(1);

                    if (shootTimer.seconds() > 5 && shotNumber == 1){
                        upperRoller.setPower(0);
                        shotsRemaining = 0;
                        shotNumber = 2;
                        flywheelState = FlywheelState.SPIN_UP;
                    } else if (shootTimer.seconds() > 3 && shotNumber == 2) {
                        upperRoller.setPower(0);
                        shotsRemaining = 0;
                        flywheelState = FlywheelState.SPIN_UP;
                    }else{
                        upperRoller.setPower(1);
                    }
                }

                break;


        }
    }


    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.SPIN_UP) {
            shotsRemaining = numberOfShots;
        }
    }
    public void spinUp(boolean readyToFly) {
        if (flywheelState == FlywheelState.IDLE) {
            spinningUp = readyToFly;
        }
    }


    public boolean isBusy() {
        return flywheelState != FlywheelState.SPIN_UP;
    }
}

