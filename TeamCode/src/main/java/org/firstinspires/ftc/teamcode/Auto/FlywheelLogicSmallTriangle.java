package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class FlywheelLogicSmallTriangle {
    private DcMotorEx shooter;
    private CRServo upperRoller;
    private CRServo lowerRoller;
    private Servo turret;
    private Servo gate;
    private Servo leftHood;
    private Servo rightHood;
    private Servo spindexer;
    private Servo light;
    private Limelight3A limelight;
    private ElapsedTime stateTimer = new ElapsedTime();
    private  ElapsedTime shootTimer = new ElapsedTime();
    private Servo light1;
    private Servo light2;
    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        SORT,
        LAUNCH;
    }
    private FlywheelState flywheelState;

    private int shotsRemaining = 0;
    private int shotNumber = 1;
    private double flywheelVelocity = 0;
    private double MIN_FLYWHEEL_RPM = 1400;
    private double TARGET_FLYWHEEL_RPM = 1550;
    private double FLYWHEEL_MAX_SPINUP_TIME = 2.25;
    double P = 52;
    double F = 14.04;


    boolean spinningUp = false;

    private String pattern = "";


    public void init(HardwareMap hwMap) {
        leftHood = hwMap.get(Servo.class, "leftHood");
        rightHood = hwMap.get(Servo.class, "rightHood");
        upperRoller = hwMap.get(CRServo.class, "upperRoller");
        lowerRoller = hwMap.get(CRServo.class, "lowerRoller");
        turret = hwMap.get(Servo.class, "turret");
        gate = hwMap.get(Servo.class, "gate");
        light = hwMap.get(Servo.class, "light1");
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        spindexer = hwMap.get(Servo.class, "spindexer");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelState = FlywheelState.IDLE;

        shooter.setVelocity(0);
        spindexer.setPosition(0.22);
        leftHood.setPosition(0.915);
        rightHood.setPosition(0.085);



    }

    public void startLimeLight(){
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public void getID(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int tagID = fr.getFiducialId();
                if (tagID == 21) {
                    pattern = "GPP";
                }
                else if (tagID == 22){
                    pattern = "PGP";
                }
                else if (tagID == 23){
                    pattern = "PPG";
                }


            }
        }
    }



    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (spinningUp) {

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                shooter.setVelocity(TARGET_FLYWHEEL_RPM);
                gate.setPosition(0.7);
                turret.setPosition(0.07);
                lowerRoller.setPower(1);
                upperRoller.setPower(0);
                if(shotsRemaining > 0){
                    stateTimer.reset();
                    flywheelState = FlywheelState.SORT;
                }

                break;
            case SORT:
                if(shotNumber == 1){
                    if (pattern.equals("GPP")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.22);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.15);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.3);
                        }
                    } else if (pattern.equals("PGP")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.15);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.22);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.3);
                        }
                    } else if (pattern.equals("PPG")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.15);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.3);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.22);
                        }
                    }
                } else if (shotNumber == 2) {
                    if (pattern.equals("GPP")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.15);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.22);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.3);
                        }
                    } else if (pattern.equals("PGP")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.22);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.15);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.3);
                        }
                    } else if (pattern.equals("PPG")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.22);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.3);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.15);
                        }
                    }
                } else if(shotNumber == 3) {
                    if (pattern.equals("GPP")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.22);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.15);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.3);
                        }
                    } else if (pattern.equals("PGP")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.15);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.22);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.3);
                        }
                    } else if (pattern.equals("PPG")) {
                        if (shotsRemaining == 3) {
                            spindexer.setPosition(0.15);
                        } else if (shotsRemaining == 2) {
                            spindexer.setPosition(0.3);
                        } else if (shotsRemaining == 1) {
                            spindexer.setPosition(0.22);
                        }
                    }
                }

                if(shooter.getVelocity() > MIN_FLYWHEEL_RPM){
                    stateTimer.reset();
                    shootTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:

                if (shooter.getVelocity() > MIN_FLYWHEEL_RPM || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {

                        gate.setPosition(0.2);

                        lowerRoller.setPower(1);




                        if (shootTimer.seconds() > 1.6){
                            upperRoller.setPower(0);
                            shotsRemaining--;
                            if (shotsRemaining > 0) {
                                stateTimer.reset();
                                flywheelState = FlywheelState.SORT;
                            }
                            else if(shotsRemaining == 0){
                                shotNumber++;
                                flywheelState = FlywheelState.IDLE;

                            }
                        } else {
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

