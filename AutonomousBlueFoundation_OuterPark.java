package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.graphics.Color;
import android.app.Activity;
import android.view.View;
import java.lang.annotation.Target;
import java.util.Timer;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import android.app.Activity;

@Autonomous

public class AutonomousBlueFoundation_OuterPark extends LinearOpMode {
    
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFR;
    private DcMotor armAngle;
    private DcMotor armSlide;
    private Servo servoLS;
    private Servo servoRS;
    private Servo servoLR;
    private Servo servoRR;
    private CRServo intakeL;
    private CRServo intakeR;
    private Servo angleL;
    private Servo angleR;
    int currentArm = 0, currentFL = 0, currentBL = 0, currentFR = 0, currentBR = 0;
    
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    
    public void motors (String direction, int distance, double speed) {
        if (direction.equals("arm")) {
            while ((Math.abs(armAngle.getCurrentPosition() - currentArm) < Math.abs(distance)) && opModeIsActive()) {
                armAngle.setTargetPosition(currentArm + distance);
                armAngle.setPower(speed);
            }
            currentArm = armAngle.getCurrentPosition();
        }
        
        else if (direction.equals("front")) {
            while ((Math.abs(motorFL.getCurrentPosition() - currentFL) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(currentFL - distance);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(currentBL - distance);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(currentFR + distance);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(currentBR + distance);
                motorBR.setPower(speed);
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
            currentFL = motorFL.getCurrentPosition();
            currentBL = motorBL.getCurrentPosition();
            currentFR = motorFR.getCurrentPosition();
            currentBR = motorBR.getCurrentPosition();
        }
        
        else if (direction.equals("back")) {
            while ((Math.abs(motorFL.getCurrentPosition() - currentFL) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(currentFL + distance);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(currentBL + distance);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(currentFR - distance);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(currentBR - distance);
                motorBR.setPower(speed);
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
            currentFL = motorFL.getCurrentPosition();
            currentBL = motorBL.getCurrentPosition();
            currentFR = motorFR.getCurrentPosition();
            currentBR = motorBR.getCurrentPosition();
        }
        
        else if (direction.equals("right")) {
            while ((Math.abs(motorFL.getCurrentPosition() - currentFL) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(currentFL + distance);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(currentBL - distance);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(currentFR + distance);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(currentBR - distance);
                motorBR.setPower(speed);
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
            currentFL = motorFL.getCurrentPosition();
            currentBL = motorBL.getCurrentPosition();
            currentFR = motorFR.getCurrentPosition();
            currentBR = motorBR.getCurrentPosition();
        }
        
        else if (direction.equals("left")) {
            while ((Math.abs(motorFL.getCurrentPosition() - currentFL) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(currentFL - distance);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(currentBL + distance);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(currentFR - distance);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(currentBR + distance);
                motorBR.setPower(speed);
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
            currentFL = motorFL.getCurrentPosition();
            currentBL = motorBL.getCurrentPosition();
            currentFR = motorFR.getCurrentPosition();
            currentBR = motorBR.getCurrentPosition();
        }
        
        else if (direction.equals("turn_left")) {
            while ((Math.abs(motorFL.getCurrentPosition() - currentFL) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(currentFL - distance);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(currentBL - distance);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(currentFR - distance);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(currentBR - distance);
                motorBR.setPower(speed);
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
            currentFL = motorFL.getCurrentPosition();
            currentBL = motorBL.getCurrentPosition();
            currentFR = motorFR.getCurrentPosition();
            currentBR = motorBR.getCurrentPosition();
        }
        
        else if (direction.equals("turn_right")) {
            while ((Math.abs(motorFL.getCurrentPosition() - currentFL) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(currentFL + distance);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(currentBL + distance);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(currentFR + distance);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(currentBR + distance);
                motorBR.setPower(speed);
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
            currentFL = motorFL.getCurrentPosition();
            currentBL = motorBL.getCurrentPosition();
            currentFR = motorFR.getCurrentPosition();
            currentBR = motorBR.getCurrentPosition();
        }
    }
    
    @Override
    public void runOpMode() {
        
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        armAngle = hardwareMap.get(DcMotor.class, "armAngle");
        armSlide = hardwareMap.get(DcMotor.class, "armSlide");
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        servoLS = hardwareMap.get(Servo.class, "servoLS");
        servoRS = hardwareMap.get(Servo.class, "servoRS");
        servoLR = hardwareMap.get(Servo.class, "servoLR");
        servoRR = hardwareMap.get(Servo.class, "servoRR");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        angleL = hardwareMap.get(Servo.class, "angleL");
        angleR = hardwareMap.get(Servo.class, "angleR");
        
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
        armAngle.setTargetPosition(0);
        armSlide.setTargetPosition(0);
        
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        
        waitForStart();
        
        while (opModeIsActive()) {
            servoLS.setPosition(.6);
            servoRS.setPosition(.4);
            angleL.setPosition(.5);
            angleR.setPosition(.5);
            
            //Grab the foundation
            motors("right", 600, .2);
            motors("back", 2100, .5);
            //motors("back", 400, .2);
            servoLS.setPosition(0);
            servoRS.setPosition(1);
            
            //Bring the foundation to the depot
            //motors("front", 270, .3);
            motors("front", 2400, .4);
            servoLS.setPosition(.6);
            servoRS.setPosition(.4);
            
            //Park under the bridge
            motors("left", 2200, .4);
            motors("back", 1100, .4);
            motors("right", 1400, .4);
            motors("front", 1500, .4);
            motors("left", 2000, .4);
            motors("right", 1000, 0);
        }
    }
}
