package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import android.graphics.Color;

@TeleOp

public class MotorTest extends LinearOpMode{
    
    private DcMotor motor;
    
    
    
    
   
    
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
       
        
        
        
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       
        
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
      
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            /*(Color.RGBToHSV((int) (sensorColor.red() * 255),
                (int) (sensorColor.green() * 255),
                (int) (sensorColor.blue() * 255),
                hsvValues);
            
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Distance", sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("In", in);
            telemetry.update();
            */
           
            if(gamepad2.x) {
                motor.setPower(1);
            } 
            } else {
                intakeFL.setPower(0);
            }
            
            
        
       
        }
    }
}
