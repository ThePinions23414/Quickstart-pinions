package org.firstinspires.ftc.teamcode.TeleOp;



import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitch {
    private DigitalChannel lLimitSwitch;
    private DigitalChannel rLimitSwitch;
    public void init(HardwareMap hwMap){
        lLimitSwitch = hwMap.get(DigitalChannel.class, "leftLimitSwitch");
        lLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rLimitSwitch = hwMap.get(DigitalChannel.class, "rightLimitSwitch");
        rLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isLeftLimitSwitchClosed(){
        return !lLimitSwitch.getState();
    }
    public boolean isRightLimitSwitchClosed(){
        return !rLimitSwitch.getState();
    }

}

