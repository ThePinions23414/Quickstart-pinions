package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public class TurretMechanismV2 {
    private DcMotorEx turret;
    private Limelight3A limelight;
    private double kP = 0.0001;
    private double kD = 0.000000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.5;
    private final double MAX_POWER = 1;
    private final double ZERO_POWER = 0;
    private double power = 0;
    private double desiredPower;
    private final ElapsedTime timer = new ElapsedTime();
    LimitSwitch limitSwitch = new LimitSwitch();




    public void init(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limitSwitch.init(hwMap);

    }

    public void setGoalXOffset (int goalXOffset){
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
            turret.setPower(0);
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
           desiredPower = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
//            if(limitSwitch.isLeftLimitSwitchClosed()){
//                power = Range.clip(pTerm + dTerm, ZERO_POWER, MAX_POWER);
//            }else if(limitSwitch.isRightLimitSwitchClosed()){
//                power = Range.clip(pTerm + dTerm, -MAX_POWER, ZERO_POWER);
//            }else{
//                power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
//            }
            if (limitSwitch.isLeftLimitSwitchClosed() && desiredPower < 0) {
                // Trying to move further left — block it
                power = 0;
            }
            else if (limitSwitch.isRightLimitSwitchClosed() && desiredPower > 0) {
                // Trying to move further right — block it
                power = 0;
            }
            else {
                // Safe to move
                power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
            }

        }

        // magnetic limit switch safety check

        turret.setPower(power);
        lastError = error;

    }


}

