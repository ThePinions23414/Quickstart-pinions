package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimeLightClass {
    private Limelight3A limelight;

    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
    }



    public void update(){
        LLResult result = limelight.getLatestResult();

    }

    public void startLimeLight(int pipeline){
        limelight.start();
        limelight.pipelineSwitch(pipeline);
    }

    public LLResult getSpecificId(int targetTagId){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if(fr.getFiducialId() == targetTagId){
                    return result;
                }
            }
        }
        return null;

    }

}

