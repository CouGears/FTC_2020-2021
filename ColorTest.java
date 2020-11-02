package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;

@TeleOp

public class ColorTest extends LinearOpMode {
    private ColorSensor sensorColor1, sensorColor2;
    
    float hsvValues1[] = {0F, 0F, 0F};
    float hsvValues2[] = {0F, 0F, 0F};
    final float values[] = hsvValues1;
    final double SCALE_FACTOR = 255;
    
    @Override
    public void runOpMode() {
        sensorColor1 = hardwareMap.get(ColorSensor.class, "sensorColor1");
        sensorColor2 = hardwareMap.get(ColorSensor.class, "sensorColor2");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            Color.RGBToHSV((int) (sensorColor1.red() * 255),
                (int) (sensorColor1.green() * 255),
                (int) (sensorColor1.blue() * 255),
                hsvValues1);
            Color.RGBToHSV((int) (sensorColor2.red() * 255),
                (int) (sensorColor2.green() * 255),
                (int) (sensorColor2.blue() * 255),
                hsvValues2);
            
            telemetry.addData("Hue1", hsvValues1[0]);
            telemetry.addData("Hue2", hsvValues2[0]);
            telemetry.update();
        }
    }
}