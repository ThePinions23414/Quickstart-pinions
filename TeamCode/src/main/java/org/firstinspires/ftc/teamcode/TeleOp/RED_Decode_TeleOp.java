package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name = "RED_Decode_TeleOp")
public class RED_Decode_TeleOp extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor fL = hardwareMap.dcMotor.get("frontLeft");
        DcMotor bL = hardwareMap.dcMotor.get("backLeft");
        DcMotor fR = hardwareMap.dcMotor.get("frontRight");
        DcMotor bR = hardwareMap.dcMotor.get("backRight");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        CRServo lLifter = hardwareMap.get(CRServo.class, "leftLifter");
        CRServo rLifter = hardwareMap.get(CRServo.class, "rightLifter");
        CRServo lowerRoller = hardwareMap.get(CRServo.class, "lowerRoller");
        CRServo upperRoller = hardwareMap.get(CRServo.class, "upperRoller");
        Servo turret = hardwareMap.get(Servo.class, "turret");
        Servo spindexer = hardwareMap.get(Servo.class, "spindexer");
        Servo lHood = hardwareMap.get(Servo.class, "leftHood");
        Servo rHood = hardwareMap.get(Servo.class, "rightHood");
        Servo gate = hardwareMap.get(Servo.class, "gate");
        Servo light1 = hardwareMap.get(Servo.class, "light1");
        Servo light2 = hardwareMap.get(Servo.class, "light2");
        Servo light3 = hardwareMap.get(Servo.class, "light3");


        ColorSensor intakeColorSensor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");
        ColorSensor shooterColorSensor = hardwareMap.get(ColorSensor.class, "shooterColorSensor");
        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        double driveSpeed = 1;
        double redValue = 0;
        double greenValue = 0;
        double blueValue = 0;
        double velocity = 0;
        double idleVelocity = 300;
        double slot1Position = 0.15;
        double slot2Position = 0.22;
        double slot3Position = 0.3;
        double F = 0;
        double P = 0;
        int ballCount = 0;
        int ballTimerNumber = 1;
        String slot1Color = "empty";
        String slot2Color = "empty";
        String slot3Color = "empty";
        String colorShot = "none";
        boolean spindexerOn = false;
        boolean someoneTurnedOffSpindexer = false;
        boolean touchSensorPressed = false;
        boolean override = false;
        boolean thirdBall = false;
        boolean shooterActivated = false;
        boolean ballIsLaunching = false;
        boolean ballHasLaunched = false;
        boolean ball1ReadyForEntry = false;
        boolean ball2ReadyForEntry = false;
        boolean ball3ReadyForEntry = false;






        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);





        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);


        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime rumbleTimer = new ElapsedTime();
        ElapsedTime spindexerTimer = new ElapsedTime();
        ElapsedTime ball1TravelTimer = new ElapsedTime();
        ElapsedTime ball2TravelTimer = new ElapsedTime();
        ElapsedTime ball3TravelTimer = new ElapsedTime();

        LimitSwitch limitSwitch = new LimitSwitch();
        limitSwitch.init(hardwareMap);

//        double[] numbers = {0, 0.2, 0.4, 0.6, 0.8};
//
//        for (double number : numbers) {
//            number = number - 0.2;
//        }
//
//        int smallest = 0;
//
//
//        for (int i = 1; i < numbers.length; i++) {
//            if (numbers[i] < smallest) {
//                smallest = (int) numbers[i];
//
//            }

        limelight.pipelineSwitch(1);


        limelight.start();





        waitForStart();



        rumbleTimer.reset();
        spindexerTimer.reset();
        ball1TravelTimer.reset();
        ball2TravelTimer.seconds();
        ball3TravelTimer.seconds();



        spindexer.setPosition(slot1Position);
        gate.setPosition(0.7);
        turret.setPosition(0.07);
        lowerRoller.setPower(1);





        if (isStopRequested()) {
            return;


        }


        while (opModeIsActive()) {

//              if(gamepad1.dpad_up){
//                  lHood.setPosition(0.875);
//                  rHood.setPosition(0.125);
//              }
//              if(gamepad1.dpad_down){
//                  lHood.setPosition(1);
//                  rHood.setPosition(0);
//              }
//
//
//
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


            if (rumbleTimer.seconds() > 110 && rumbleTimer.seconds() < 119) {
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }


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




            if (spindexerTimer.seconds() > 75 && !someoneTurnedOffSpindexer) {
                spindexerOn = true;
            }
            if(rumbleTimer.seconds() > 75 && rumbleTimer.seconds() < 77){
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }





            if (touchSensor.isPressed()) {
                touchSensorPressed = true;
                greenValue = intakeColorSensor.green();
                blueValue = intakeColorSensor.blue();
            }
            if (touchSensorPressed == true && !touchSensor.isPressed()) {
                ballCount++;
                if (greenValue > blueValue) {
                    if (ballCount == 1) {
                        slot1Color = "green";
                        light1.setPosition(0.5);
                    } else if (ballCount == 2) {
                        slot2Color = "green";
                        light2.setPosition(0.5);
                    } else if (ballCount == 3) {
                        slot3Color = "green";
                        light3.setPosition(0.5);
                    }

                } else if (greenValue < blueValue) {
                    if (ballCount == 1) {
                        slot1Color = "purple";
                        light1.setPosition(0.7);
                    } else if (ballCount == 2) {
                        slot2Color = "purple";
                        light2.setPosition(0.7);
                    } else if (ballCount == 3) {
                        slot3Color = "purple";
                        light3.setPosition(0.7);
                    }

                }
                if(ballCount == 1){
                    spindexer.setPosition(slot1Position);
                    ball1TravelTimer.reset();
                    if(spindexerOn){
                        ball1ReadyForEntry = true;
                    }
                }
                else if(ballCount == 2){
                    ball2TravelTimer.reset();
                    if(spindexerOn){
                        ball2ReadyForEntry = true;
                    }
                }
                else if(ballCount == 3){
                    ball3TravelTimer.reset();
                    if(spindexerOn){
                        ball3ReadyForEntry = true;
                    }
                }
                touchSensorPressed = false;
            }
            if(ballCount == 2 && touchSensor.isPressed() && !spindexerOn) {
                ballCount++;
                greenValue = intakeColorSensor.green();
                blueValue = intakeColorSensor.blue();
                if (greenValue > blueValue) {
                    if (ballCount == 1) {
                        slot1Color = "green";
                        light1.setPosition(0.5);
                    } else if (ballCount == 2) {
                        slot2Color = "green";
                        light2.setPosition(0.5);
                    } else if (ballCount == 3) {
                        slot3Color = "green";
                        light3.setPosition(0.5);
                    }

                } else if (greenValue < blueValue) {
                    if (ballCount == 1) {
                        slot1Color = "purple";
                        light1.setPosition(0.7);
                    } else if (ballCount == 2) {
                        slot2Color = "purple";
                        light2.setPosition(0.7);
                    } else if (ballCount == 3) {
                        slot3Color = "purple";
                        light3.setPosition(0.7);
                    }

                }


                thirdBall = true;

            }
            if(thirdBall == true && !touchSensor.isPressed()){
                ballCount--;
                thirdBall = false;
            }

            if (spindexerOn == true) {
                if (gamepad2.left_bumper) {
                    lowerRoller.setPower(1);
                    if (slot1Color.equals("green")) {
                        spindexer.setPosition(slot1Position);
                    } else if (slot2Color.equals("green")) {
                        spindexer.setPosition(slot2Position);
                    } else if (slot3Color.equals("green")) {
                        spindexer.setPosition(slot3Position);
                    }

                }

                if (gamepad2.right_bumper) {
                    lowerRoller.setPower(1);
                    if (slot1Color.equals("purple")) {
                        spindexer.setPosition(slot1Position);
                    } else if (slot2Color.equals("purple")) {
                        spindexer.setPosition(slot2Position);
                    } else if (slot3Color.equals("purple")) {
                        spindexer.setPosition(slot3Position);
                    }

                }



            }

            if(ball1ReadyForEntry && ball1TravelTimer.seconds() > 2 && spindexerOn){
                lowerRoller.setPower(1);
                spindexer.setPosition(slot2Position);
                ball1ReadyForEntry = false;
            } else if (ball2ReadyForEntry && ball2TravelTimer.seconds() > 2 && ball1TravelTimer.seconds() > 3 && spindexerOn) {
                lowerRoller.setPower(1);
                spindexer.setPosition(slot3Position);
                ball2ReadyForEntry = false;
            } else if (ball3ReadyForEntry && ball3TravelTimer.seconds() > 2 && ball1TravelTimer.seconds() > 4 && ball2TravelTimer.seconds() > 3 && spindexerOn) {
                lowerRoller.setPower(1);
                spindexer.setPosition(slot1Position);
                ball3ReadyForEntry = false;
            }

            if(shooterColorSensor.green() > shooterColorSensor.blue() && shooterColorSensor.blue() > 200 || shooterColorSensor.blue() > shooterColorSensor.green() && shooterColorSensor.green() > 200){
                if(shooterColorSensor.green() > shooterColorSensor.blue() && shooterColorSensor.blue() > 100){
                    colorShot = "green";

                }

                else if(shooterColorSensor.blue() > shooterColorSensor.green() && shooterColorSensor.green() > 100){
                    colorShot = "purple";
                }
                ballIsLaunching = true;
                ballHasLaunched = true;
            }else{
                ballIsLaunching= false;
            }

            if(ballHasLaunched && !ballIsLaunching){
                ballCount--;
                if (slot1Color.equals("green") && colorShot.equals("green")) {
                    slot1Color = "empty";
                    light1.setPosition(0);
                } else if (slot2Color.equals("green") && colorShot.equals("green")) {
                    slot2Color = "empty";
                    light2.setPosition(0);
                } else if (slot3Color.equals("green") && colorShot.equals("green")) {
                    slot3Color = "empty";
                    light3.setPosition(0);
                }
                if (slot1Color.equals("purple") && colorShot.equals("purple")) {
                    slot1Color = "empty";
                    light1.setPosition(0);
                } else if (slot2Color.equals("purple") && colorShot.equals("purple")) {
                    slot2Color = "empty";
                    light2.setPosition(0);
                } else if (slot3Color.equals("purple") && colorShot.equals("purple")) {
                    slot3Color = "empty";
                    light3.setPosition(0);
                }
                ballHasLaunched = false;

            }







            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access general information

                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());


                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                if(result.getTx() < -3){

                    if(limitSwitch.isLeftLimitSwitchClosed()){
                        telemetry.addLine("Limit Reached");
                    }else{
                        turret.setPosition(turret.getPosition() - 0.0005);
                        telemetry.addLine("Moving left");
                    }
                }

                else if(result.getTx() > -3 && result.getTx() < 1){
                    telemetry.addLine("Stopped");


                }
                else if(result.getTx() > 1) {

                    if(limitSwitch.isRightLimitSwitchClosed()){
                        telemetry.addLine("Limit Reached");
                    }else{
                        turret.setPosition(turret.getPosition() + 0.0005);
                        telemetry.addLine("Moving right");
                    }
                }




//                    if(result.getTx() < -7){
//                        telemetry.addLine("Moving left");
//                        turret.setPower(-0.25);
//                    }
//
//                    else if(result.getTx() > -7 && result.getTx() < 7){
//                        telemetry.addLine("Stopped");
//                        turret.setPower(0);
//
//                    }
//                    else if(result.getTx() > 7){
//                        telemetry.addLine("Moving right");
//                        turret.setPower(0.25);
//
//                    }




//                    else if(result.getTx() < -2 && result.getTx() > 0){
//                        telemetry.addLine("Stopped");
//                        turret.setPower(0);
//
//                    }
//                    if(!limitSwitch.isRightLimitSwitchClosed()){
//
//
//
//                    }else{
//                        turret.setPower(0);
//                    }

//                    if (gamepad2.x) {
//
//
//                        if (result.getTa() < 3.5 && result.getTa() > 2.3) {
//                            shooter.setVelocity(1150);
//                            lHood.setPosition(1);
//                            rHood.setPosition(0);
//                        }
//                        else if (result.getTa() < 2.3 && result.getTa() > 1) {
//                            shooter.setVelocity(1200);
//                            lHood.setPosition(0.95);
//                            rHood.setPosition(0.05);
//                        }
//                        else if (result.getTa() < 1 && result.getTa() > 0.38) {
//                            shooter.setVelocity(1375);
//                            lHood.setPosition(0.925);
//                            rHood.setPosition(0.075);
//                        }
//                        else if (result.getTa() < 0.38 && result.getTa() > 0.29) {
//                            shooter.setVelocity(1700);
//                            lHood.setPosition(0.915);
//                            rHood.setPosition(0.085);
//                        }
//                        else if (result.getTa() < 0.29) {
//                            shooter.setVelocity(1735);
//                            lHood.setPosition(0.915);
//                            rHood.setPosition(0.085);
//                        }
//                    }



                if (!override) {
                    if (result.getTa() < 3.5 && result.getTa() > 2.9) {
                        velocity = 1100;
                        P = 40;
                        F = 13.51;
                        lHood.setPosition(0.95);
                        rHood.setPosition(0.05);
                    } else if (result.getTa() < 2.9 && result.getTa() > 2.3) {
                        velocity = 1200;
                        P = 64.05;
                        F = 13.0604;
                        lHood.setPosition(0.95);
                        rHood.setPosition(0.05);
                    } else if (result.getTa() < 2.3 && result.getTa() > 1.7) {
                        velocity = 1250;
                        P = 62;
                        F = 13.95;
                        lHood.setPosition(0.95);
                        rHood.setPosition(0.05);
                    } else if (result.getTa() < 1.7 && result.getTa() > 1.1) {
                        velocity = 1300;
                        P = 52;
                        F = 13.96;
                        lHood.setPosition(0.95);
                        rHood.setPosition(0.05);
                    } else if (result.getTa() < 1.1 && result.getTa() > 0.9) {
                        velocity = 1350;
                        P = 48;
                        F = 13.93;
                        lHood.setPosition(0.925);
                        rHood.setPosition(0.075);
                    } else if (result.getTa() < 0.9 && result.getTa() > 0.6) {
                        velocity = 1400;
                        P = 42;
                        F = 14.2;
                        lHood.setPosition(0.925);
                        rHood.setPosition(0.075);
                    } else if (result.getTa() < 0.6 && result.getTa() > 0.43) {
                        velocity = 1450;
                        P = 49;
                        F = 13.98;
                        lHood.setPosition(0.925);
                        rHood.setPosition(0.075);
                    } else if (result.getTa() < 0.43 && result.getTa() > 0.3) {
                        velocity = 1550;
                        P = 52;
                        F = 14.04;
                        lHood.setPosition(0.915);
                        rHood.setPosition(0.085);
                    } else if (result.getTa() < 0.3) {
                        velocity = 1650;
                        P = 52;
                        F = 14.04;
                        lHood.setPosition(0.915);
                        rHood.setPosition(0.085);
                    }

                }


            }else{
                telemetry.addLine("No AprilTag in sight");
            }

            if (result.getTa() == 0) {
                velocity = 1100;
                P = 40;
                F = 13.51;
                lHood.setPosition(0.95);
                rHood.setPosition(0.05);
            }












            if (gamepad2.y && !override) {
                someoneTurnedOffSpindexer = false;
                spindexerOn = true;
            }

            if (gamepad2.a) {
                someoneTurnedOffSpindexer = true;
                spindexerOn = false;
            }



            if(gamepad2.left_trigger > 0.1){
                override = true;
            }
            else if(gamepad2.left_trigger < 0.1){
                override = false;
            }

            if(ballCount < 3 || override || !spindexerOn){
                if (gamepad2.dpad_left) {
                    intake.setPower(1);

                }
                else {
                    intake.setPower(0);
                }
            }else {
                intake.setPower(0);
            }



            if(gamepad2.dpad_left){
                lowerRoller.setPower(1);
                lLifter.setPower(1);
                rLifter.setPower(-1);
            }

            if (gamepad2.dpad_right) {
                lowerRoller.setPower(0);
                lLifter.setPower(0);
                rLifter.setPower(0);
                shooter.setPower(0);
                shooter.setVelocity(0);
                gate.setPosition(0.7);
                shooterActivated = false;

                ballCount = 0;
                light1.setPosition(0);
                light2.setPosition(0);
                light3.setPosition(0);
                slot1Color = "empty";
                slot2Color = "empty";
                slot3Color = "empty";



            }
            if(gamepad2.dpad_down && override){
                velocity = 1200;
                P = 64.05;
                F = 13.0604;
                shooterActivated = true;
                lHood.setPosition(0.95);
                rHood.setPosition(0.05);
                light1.setPosition(0.4);
            }
            if(gamepad2.dpad_up && override){
                velocity = 1550;
                P = 52;
                F = 14.04;
                shooterActivated = true;
                lHood.setPosition(0.915);
                rHood.setPosition(0.085);
            }


            if(gamepad2.x && !override){
                shooterActivated = true;

            }

            if(gamepad2.a && override){
                shooterActivated = true;

            }

            if(shooterActivated){
                shooter.setVelocity(velocity);
            } else if (!shooterActivated) {
                P = 10;
                F = 18.1;
                shooter.setVelocity(idleVelocity);
            }


            if (gamepad2.bWasPressed() && !override){
                upperRoller.setPower(1);
                gate.setPosition(0.2);

            }
            else if (gamepad2.bWasReleased() && !override) {
                upperRoller.setPower(0);
            }

            if(gamepad2.ps){
                ballCount = 0;
                light1.setPosition(0);
                light2.setPosition(0);
                light3.setPosition(0);
                slot1Color = "empty";
                slot2Color = "empty";
                slot3Color = "empty";
                spindexer.setPosition(slot1Position);
                lowerRoller.setPower(1);
            }

            if(gamepad2.x && override){
                spindexer.setPosition(slot1Position);
            }
            if(gamepad2.y && override){
                spindexer.setPosition(slot2Position);
            }
            if(gamepad2.b && override){
                spindexer.setPosition(slot3Position);
            }

            telemetry.addData("Match Timer", rumbleTimer);
            telemetry.addData("ball 1 timer", ball1TravelTimer);
            telemetry.addData("ball 2 timer", ball2TravelTimer);
            telemetry.addData("ball 3 timer", ball3TravelTimer);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("slot1color", slot1Color);
            telemetry.addData("slot2color", slot2Color);
            telemetry.addData("slot3color", slot3Color);
            telemetry.addData("green",greenValue);
            telemetry.addData("blue", blueValue);
            telemetry.addData("Left Magnet state" , limitSwitch.isLeftLimitSwitchClosed());
            telemetry.addData("Right Magnet state" , limitSwitch.isRightLimitSwitchClosed());
            telemetry.update();


        }
        limelight.stop();
    }
}










