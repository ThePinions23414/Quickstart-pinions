package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public class ColorBlobMechanism {
    private DcMotorEx fL;
    private DcMotorEx fR;
    private DcMotorEx bL;
    private DcMotorEx bR;
    private Limelight3A limelight;
    private double kP = 0.0001;
    private double kD = 0.000000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.5;
    private final double MAX_POWER = 1;
    private final double REDUCED_POWER = 0.5;
    private double power = 0;
    private double desiredPower;
    private final ElapsedTime timer = new ElapsedTime();





    public void init(HardwareMap hwMap){
        fL = hwMap.get(DcMotorEx.class, "frontLeft");
        fR = hwMap.get(DcMotorEx.class, "frontRight");
        bL = hwMap.get(DcMotorEx.class, "backLeft");
        bR = hwMap.get(DcMotorEx.class, "backRight");

//        fL.setDirection(DcMotorSimple.Direction.REVERSE);
//        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void setGoalXOffset (double goalXOffset){
        goalX = goalXOffset;
    }





    public void setkP (double newKP){
        kP = newKP;
    }

    public double getkP(){
        return kP;
    }

    public void setkD (double newKD){
        kD = newKD;
    }

    public double getkD(){
        return kD;
    }


    public void resetTimer(){
        timer.reset();
    }

    public void update(LLResult result){
        double deltaTime = timer.seconds();
        timer.reset();



        if (result == null){
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
            lastError = 0;
            return;
        }

        double error = goalX - result.getTx();
        double pTerm = -error * kP;

        double dTerm = 0;
        if (deltaTime > 0){
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance){
            power = 0;
        } else{
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }
        //else{
//            if(limelight.getX() < -400 || limelight.getX() > 300){
//                desiredPower = Range.clip(pTerm + dTerm, -REDUCED_POWER, REDUCED_POWER);
//            }else{
//                desiredPower = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
//            }
        fL.setPower(-power);
        fR.setPower(-power);
        bL.setPower(-power);
        bR.setPower(-power);
        lastError = error;

    }


}

