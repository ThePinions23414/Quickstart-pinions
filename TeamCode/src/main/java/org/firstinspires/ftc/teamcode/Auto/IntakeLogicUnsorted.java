package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeLogicUnsorted {



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
    double slot1Position = 0.185;
    private enum IntakeStateUnsorted {
        IDLENOSORT,
        INTAKENOSORT,
        HOLDNOSORT;

    }
    private IntakeLogicUnsorted.IntakeStateUnsorted intakeState;

    public void init(HardwareMap hwMap) {
        spindexer = hwMap.get(Servo.class, "spindexer");
        intake = hwMap.get(DcMotor.class, "intake");
        lowerRoller = hwMap.get(CRServo.class, "lowerRoller");
        lLifter = hwMap.get(CRServo.class, "leftLifter");
        rLifter = hwMap.get(CRServo.class, "rightLifter");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeState = IntakeLogicUnsorted.IntakeStateUnsorted.IDLENOSORT;

        spindexer.setPosition(0);


    }



    public void update() {
        switch (intakeState) {
            case IDLENOSORT:

                if (ballsRemaining == 0) {
                    spindexer.setPosition(slot1Position);
                    stateTimer.reset();
                    intakeTimer.reset();
                    intakeState = IntakeLogicUnsorted.IntakeStateUnsorted.INTAKENOSORT;
                }
                break;
            case INTAKENOSORT:


                if(ballPickUpNumber == 1){
                    intake.setPower(1);
                    lowerRoller.setPower(1);
                    lLifter.setPower(1);
                    rLifter.setPower(-1);
                    if(stateTimer.seconds() > 2){
                        lLifter.setPower(0);
                        rLifter.setPower(0);
                        stateTimer.reset();
                        intakeState = IntakeLogicUnsorted.IntakeStateUnsorted.HOLDNOSORT;
                    }
                } else if (ballPickUpNumber == 2) {
                    intake.setPower(1);
                    lowerRoller.setPower(1);
                    lLifter.setPower(1);
                    rLifter.setPower(-1);
                    if(stateTimer.seconds() > 2.35){
                        lLifter.setPower(0);
                        rLifter.setPower(0);
                        stateTimer.reset();
                        intakeState = IntakeLogicUnsorted.IntakeStateUnsorted.HOLDNOSORT;
                    }
                }


                break;
            case HOLDNOSORT:
                if (ballsRemaining == 1) {
                    if (stateTimer.seconds() > 0.7) {
                        spindexer.setPosition(slot1Position);
                        ballsRemaining -= 1;

                    }
                } else if (ballsRemaining == 1) {
                        if (stateTimer.seconds() > 0.7) {
                            spindexer.setPosition(slot1Position);
                            ballsRemaining -= 1;

                        }
                } else if (ballsRemaining == 3) {
                    if (stateTimer.seconds() > 0.5) {
                        lLifter.setPower(1);
                        rLifter.setPower(-1);
                        if (stateTimer.seconds() > 2) {
                            spindexer.setPosition(slot1Position);
                            ballsRemaining--;
                        }
                    }
                }
                break;

        }
    }

    void intakeBalls(int numberOfBalls) {
        if (intakeState == IntakeLogicUnsorted.IntakeStateUnsorted.IDLENOSORT) {
            ballsRemaining = numberOfBalls;
        }
    }

    public boolean isStillGoing() {

        return intakeState != IntakeLogicUnsorted.IntakeStateUnsorted.IDLENOSORT;
    }
}
