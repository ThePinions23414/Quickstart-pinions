package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeLogicNotSorted {



    private Servo spindexer;
    private DcMotor intake;
    private CRServo lowerRoller;
    private CRServo lLifter;
    private CRServo rLifter;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    public boolean readyToIntake = false;
    int ballsRemaining = 0;
    int ballPickUpNumber = 1;
    double slot1Position = 0.32;
    private enum IntakeState {
        IDLE,
        INTAKE,


    }
    private IntakeState intakeState;

    public void init(HardwareMap hwMap) {
        spindexer = hwMap.get(Servo.class, "spindexer");
        intake = hwMap.get(DcMotor.class, "intake");
        lowerRoller = hwMap.get(CRServo.class, "lowerRoller");
        lLifter = hwMap.get(CRServo.class, "leftLifter");
        rLifter = hwMap.get(CRServo.class, "rightLifter");

        intakeState = IntakeState.IDLE;

        spindexer.setPosition(0);


    }



    public void update() {
        switch (intakeState) {
            case IDLE:

                if (ballsRemaining > 0) {
                    spindexer.setPosition(slot1Position);
                    stateTimer.reset();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE;
                }
                break;
            case INTAKE:
                if(stateTimer.seconds() > 2.75){
                    ballsRemaining = 0;
                    stateTimer.reset();
                    intakeState = IntakeState.IDLE;
                }


                break;
        }
    }

    public void intakeBalls(int numberOfBalls) {
        if (intakeState == IntakeState.IDLE) {
            ballsRemaining = numberOfBalls;
        }
    }

    public boolean isStillGoing() {

        return intakeState != IntakeState.IDLE;
    }
}

