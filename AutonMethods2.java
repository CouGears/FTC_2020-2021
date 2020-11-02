package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class AutonMethods {
    
    public AutonMethods() {
        
    }
    
    static DcMotor motorFL, motorBL, motorBR, motorFR, intakeFL, intakeFR;
    static Servo servoLS, servoRS, servoLR, servoRR, angleL, angleR;
    static CRServo intakeML, intakeMR;
    HardwareMap map;
    Telemetry tele;
    
    private double speed;
    private boolean clampDown = true;
    public int counter = 0;
    
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
	
    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;
    
    public void motors (String direction, int distance) {
        changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        if (direction.equals("front")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 25);
                motorBL.setTargetPosition(-distance - 25);
                motorFR.setTargetPosition(distance + 25);
                motorBR.setTargetPosition(distance + 25);
                speed(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("back")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 25);
                motorBL.setTargetPosition(distance + 25);
                motorFR.setTargetPosition(-distance - 25);
                motorBR.setTargetPosition(-distance - 25);
                speed(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("right")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 25);
                motorBL.setTargetPosition(-distance - 25);
                motorFR.setTargetPosition(distance + 25);
                motorBR.setTargetPosition(-distance - 25);
                speed(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("left")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 25);
                motorBL.setTargetPosition(distance + 25);
                motorFR.setTargetPosition(-distance - 25);
                motorBR.setTargetPosition(distance + 25);
                speed(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_left")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                runWithImu(distance, direction);
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_right")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                runWithImu(distance, direction);
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("stop")) {
            motorFL.setTargetPosition(motorFL.getCurrentPosition());
            motorBL.setTargetPosition(motorBL.getCurrentPosition());
            motorFR.setTargetPosition(motorFR.getCurrentPosition());
            motorBR.setTargetPosition(motorBR.getCurrentPosition());
            speed(0);
        }
        
        if (!(Math.abs(motorFL.getCurrentPosition()) < distance) && !(direction.equals("turn_right") || direction.equals("turn_left"))) {
            counter++;
            changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    
    public void speedIncrement(int distance) {
        if (distance < 100) speed = .2;
        else if (distance < 200) speed = .25;
        else if (distance < 300) speed = .3;
        else if (distance < 400) speed = .35;
        else if (distance < 500) speed = .4;
        else if (distance < 600) speed = .45;
        else speed = .5;
    }
    
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.map = map;
        this.tele = tele;
        
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorFR = map.get(DcMotor.class, "motorFR");
        motorBR = map.get(DcMotor.class, "motorBR");
        intakeFL = map.get(DcMotor.class, "intakeFL");
        intakeFR = map.get(DcMotor.class, "intakeFR");
        //sensorColor = map.get(ColorSensor.class, "sensorColorRange");
        servoLS = map.get(Servo.class, "servoLS");
        servoRS = map.get(Servo.class, "servoRS");
        servoLR = map.get(Servo.class, "servoLR");
        servoRR = map.get(Servo.class, "servoRR");
        
        changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        
        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
        
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());
		
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        
        tele.addData(">", "Gyro Calibrating. Do Not Move!");
        tele.update();
        
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    
    public static void changeRunMode(DcMotor.RunMode runMode) {
        motorFL.setMode(runMode);
        motorBL.setMode(runMode);
        motorFR.setMode(runMode);
        motorBR.setMode(runMode);
    }
    
    public void speed(double in) {
        motorFL.setPower(in);
        motorBL.setPower(in);
        motorFR.setPower(in);
        motorBR.setPower(in);
    }
    
    public void servoClamp() {
        if (clampDown) {
            servoLS.setPosition(.6);
            servoRS.setPosition(.4);
        }
        
        else {
            servoLS.setPosition(0);
            servoRS.setPosition(1);
        }
        
        clampDown = !clampDown;
        counter++;
    }
    
    public void sleep(long sleep) {
        try {
            Thread.sleep(sleep);
        }
        catch (InterruptedException e) {
            tele.addLine("Failed Sleep");
            tele.update();
        }
        
        counter++;
    }
    
    public void intake(String direction) {
        if (direction.equals("in")) {
            intakeFL.setPower(-.5);
            intakeFR.setPower(.5);
        }
        
        else {
            intakeFL.setPower(.5);
            intakeFR.setPower(-.5);
        }
        
        counter++;
    }
    
    public void intakeOff() {
        intakeFL.setPower(0);
        intakeFR.setPower(0);
        
        counter++;
    }
	
	public void runWithImu(int angle, String direction) {
        tele.addData("Gyro:", Math.round(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)));
        tele.addData("Angle:", angle);
        tele.addData("Case:", angle != Math.round(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)));
        tele.addData("Motor Goal:", motorFL.getTargetPosition());
        tele.addData("Motor Current:", motorFL.getCurrentPosition());
        tele.update();
        
		if (angle != Math.round(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle))) {
			if (direction.equals("turn_right")) {
				motorFL.setTargetPosition(motorFL.getCurrentPosition() + 20000);
				motorBL.setTargetPosition(motorBL.getCurrentPosition() + 20000);
				motorFR.setTargetPosition(motorFR.getCurrentPosition() + 20000);
				motorBR.setTargetPosition(motorBR.getCurrentPosition() + 20000);
                speed(.5);
			}
			
			else if (direction.equals("turn_left")) {
				motorFL.setTargetPosition(motorFL.getCurrentPosition() - 20000);
				motorBL.setTargetPosition(motorBL.getCurrentPosition() - 20000);
				motorFR.setTargetPosition(motorFR.getCurrentPosition() - 20000);
				motorBR.setTargetPosition(motorBR.getCurrentPosition() - 20000);
                speed(.5);
			}
		}
		
		else {
			motorFL.setTargetPosition(motorFL.getCurrentPosition());
			motorBL.setTargetPosition(motorBL.getCurrentPosition());
			motorFR.setTargetPosition(motorFR.getCurrentPosition());
			motorBR.setTargetPosition(motorBR.getCurrentPosition());
			speed(0);
			
			tele.addData("Case:", angle != Math.round(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)));
			tele.update();
			
			counter++;
			changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
	}
}
