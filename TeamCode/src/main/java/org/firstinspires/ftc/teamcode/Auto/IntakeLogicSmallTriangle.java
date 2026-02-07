package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeLogicSmallTriangle {



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
    private enum IntakeState {
        IDLE,
        INTAKE,
        SPINDEX;

    }
    private IntakeLogicSmallTriangle.IntakeState intakeState;

    public void init(HardwareMap hwMap) {
        spindexer = hwMap.get(Servo.class, "spindexer");
        intake = hwMap.get(DcMotor.class, "intake");
        lowerRoller = hwMap.get(CRServo.class, "lowerRoller");
        lLifter = hwMap.get(CRServo.class, "leftLifter");
        rLifter = hwMap.get(CRServo.class, "rightLifter");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeState = IntakeLogicSmallTriangle.IntakeState.IDLE;

        spindexer.setPosition(0);


    }



    public void update() {
        switch (intakeState) {
            case IDLE:

                if (ballsRemaining > 0) {
                    spindexer.setPosition(0.15);
                    stateTimer.reset();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE;
                }
                break;
            case INTAKE:


                if(ballPickUpNumber == 1){
                    intake.setPower(1);
                    lowerRoller.setPower(1);
                    lLifter.setPower(1);
                    rLifter.setPower(-1);
                    if(stateTimer.seconds() > 1.95){
                        lLifter.setPower(0);
                        rLifter.setPower(0);
                        stateTimer.reset();
                        intakeState = IntakeState.SPINDEX;
                    }
                } else if (ballPickUpNumber == 2) {
                    intake.setPower(1);
                    lowerRoller.setPower(1);
                    lLifter.setPower(1);
                    rLifter.setPower(-1);
                    if(stateTimer.seconds() > 2.15){
                        lLifter.setPower(0);
                        rLifter.setPower(0);
                        stateTimer.reset();
                        intakeState = IntakeState.SPINDEX;
                    }
                }


                break;
            case SPINDEX:
                if (ballsRemaining == 3) {
                    if (stateTimer.seconds() > 0.65) {
                        spindexer.setPosition(0.22);
                        ballsRemaining--;

                    }
                }else if (ballsRemaining == 2) {
                    if(stateTimer.seconds() > 0.75){
                        lLifter.setPower(1);
                        rLifter.setPower(-1);
                        if(stateTimer.seconds() > 2){
                            spindexer.setPosition(0.3);
                            ballsRemaining--;
                        }
                    }


                } else if (ballsRemaining == 1) {

                    if(stateTimer.seconds() > 3.5){
                        spindexer.setPosition(0.15);
                        ballsRemaining--;
                    }
                } else if (ballsRemaining == 0) {

                    stateTimer.reset();
                    ballPickUpNumber = 2;
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

        return intakeState != IntakeLogicSmallTriangle.IntakeState.IDLE;
    }
}

