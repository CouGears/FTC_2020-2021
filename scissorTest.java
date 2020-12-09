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

public class scissorTest extends LinearOpMode {
public DcMotor motor, scissorMotor;
private static Servo frontScissor, backScissor;
    @Override
    public void runOpMode() {
        AutonMethods_Copy test = new AutonMethods_Copy();
        //note - this is according to front orientation - front is in the front and back is in the back 
        //also these should be configured accordingly
        
        frontScissor = hardwareMap.get(Servo.class, "frontScissor");
        backScissor = hardwareMap.get(Servo.class,  "backScissor");
        scissorMotor = hardwareMap.get(DcMotor.class,  "scissorMotor");
        scissorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        scissorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");
        
        telemetry.update();
        
        waitForStart();
        boolean s = false;
        while (opModeIsActive()) {
        
   
            if(gamepad2.x && s == false) {
                frontScissor.setPosition(1);
                test.sleep(100);
                scissorMotor.setPower(1);
                test.sleep(160);
                scissorMotor.setPower(0);
                telemetry.addData("Status", "x");
        
                telemetry.update();
                s = true;
            } 
            else if(gamepad2.y && s == true) {
                frontScissor.setPosition(.75);
                test.sleep(100);
                scissorMotor.setPower(-1);
                test.sleep(160);
                scissorMotor.setPower(0);
                telemetry.addData("Status", "y");
        
                telemetry.update();
                s = false;
            }
            else if (gamepad2.a){
                frontScissor.setPosition(0);
                
            }
            
            
        
       
        }
    }

    //takes state
    //state of false resets
    //state of true lifts
    // public void scissorServ(boolean state){
    //     public double fStart = 0;
    //     public double bStart = 5;
    //     public double fAdd = 30;
    //     public double bAdd = 53;
    //     if(state == false)
    //     {
    //         while (frontScissor.getPosition() !== double(fStart / double(180)) && backScissor.getPosition() !== double(bStart / double(180)))
    //         {
    //             frontScissor.setPosition(double(fStart / double(180)));
    //             backScissor.setPosition(double(bStart / double(180)));
    //         }
    //     }
    //     else
    //     {
    //         while (frontScissor.getPosition() !== double((fStart + fAdd) / double(180)) && backScissor.getPosition() !== double((bStart + bAdd) / double(180)))
    //         {
    //             frontScissor.setPosition(double((fStart + fAdd) / double(180)));
    //             backScissor.setPosition(double((bStart + bAdd) / double(180)));
    //         }    
    //     }  
    // }
    
}
