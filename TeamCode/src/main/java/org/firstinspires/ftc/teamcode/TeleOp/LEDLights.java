package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class LEDLights {

    private LED redLight1;
    private LED greenLight1;
    private LED redLight2;
    private LED greenLight2;

    public void init(HardwareMap hwmap){
        redLight1 = hwmap.get(LED.class, "led_red_1");
        greenLight1 = hwmap.get(LED.class, "led_green_1");
        redLight2 = hwmap.get(LED.class, "led_red_2");
        greenLight2 = hwmap.get(LED.class, "led_green_2");
    }

    public void setRedLed1(boolean isOn){
        if (isOn){
            redLight1.on();
        }
        else{
            redLight1.off();
        }
    }

    public void setGreenLed1(boolean isOn){
        if (isOn){
            greenLight1.on();
        }
        else{
            greenLight1.off();
        }
    }

    public void setRedLed2(boolean isOn){
        if (isOn){
            redLight2.on();
        }
        else{
            redLight2.off();
        }
    }

    public void setGreenLed2(boolean isOn){
        if (isOn){
            greenLight2.on();
        }
        else{
            greenLight2.off();
        }
    }
}
