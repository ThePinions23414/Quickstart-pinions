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
    private CRServo turret;
    private Limelight3A limelight;
    private double kP = 0.0001;
    private double kD = 0.000000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 3;
    private final double MAX_POWER = 1;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();
    LimitSwitch limitSwitch = new LimitSwitch();




    public void init(HardwareMap hwMap){
        turret = hwMap.get(CRServo.class, "turret");
        limitSwitch.init(hwMap);

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

        if (Math.abs(error) < angleTolerance || limitSwitch.isRightLimitSwitchClosed()|| limitSwitch.isLeftLimitSwitchClosed()){
            power = 0;
        } else{
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        // magnetic limit switch safety check

        turret.setPower(power);
        lastError = error;

    }


}

