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

public class CompetitionDriving extends LinearOpMode{
    private DcMotor motorFL, motorBL, motorBR, motorFR, intakeFL, intakeFR, intakeM, intakeB;
    private Servo servoLS, servoRS, clamp, marker;
    private DistanceSensor sensorDistance;
    private ColorSensor sensorColor1, sensorColor2;
    
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    
    int x = 0;
    boolean latch1 = true, test1 = false, latch2 = true, test2 = false, latch3 = true, test3 = false, latch4 = true, test4 = false, in = false;
    
    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        intakeFL = hardwareMap.get(DcMotor.class, "intakeFL");
        intakeFR = hardwareMap.get(DcMotor.class, "intakeFR");
        intakeM = hardwareMap.get(DcMotor.class, "intakeM");
        intakeB = hardwareMap.get(DcMotor.class, "intakeB");
        servoLS = hardwareMap.get(Servo.class, "servoLS");
        servoRS = hardwareMap.get(Servo.class, "servoRS");
        clamp = hardwareMap.get(Servo.class, "clamp");
        marker = hardwareMap.get(Servo.class, "marker");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        sensorColor1 = hardwareMap.get(ColorSensor.class, "sensorColor1");
        sensorColor2 = hardwareMap.get(ColorSensor.class, "sensorColor2");
        
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeM.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeB.setDirection(DcMotorSimple.Direction.FORWARD);
        
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
            if(gamepad1.a){
                x = 0;
            }
            
            else if(gamepad1.b){
                x = 1;
            }
            
            else if(gamepad1.x){
                x = 2;
            }
            
            else if(gamepad1.y){
                x = 3;
            }
            
            if(x == 0){
                motorFL.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x)));
                motorBL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x)));
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x)));
                motorFR.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x)));
            }
            
            else if(x == 1){ 
                motorFL.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/4);
                motorBL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/4);
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/4);
                motorFR.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/4);
            }
            
            else if(x == 2){ 
                motorFL.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/8);
                motorBL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/8);
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/8);
                motorFR.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/8);
            }
            
            else if(x == 3){ 
                motorFL.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/16);
                motorBL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/16);
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/16);
                motorFR.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/16);
            }
            
            if (gamepad2.a) {
                servoLS.setPosition(0);
                servoRS.setPosition(1);
            }
            
            else {
                servoLS.setPosition(.6);
                servoRS.setPosition(.4);
            }
            
            if (gamepad2.y) {
                marker.setPosition(0);
            }
            
            else {
                marker.setPosition(.5);
            }
            
            if (gamepad2.x) {
                clamp.setPosition(1);
            }
            
            else {
                clamp.setPosition(0);
            }
            
            while (gamepad2.left_trigger > 0) {
                if (latch2) {
                    intakeFL.setPower(-1);
                    intakeFR.setPower(-1);
                    intakeM.setPower(-.8);
                    intakeB.setPower(-.8);
                    latch3 = true;
                    latch4 = false;
                    
                    if (sensorDistance.getDistance(DistanceUnit.CM) > 8) {
                        in = true;
                    }
                    
                    else {
                        in = false;
                    }
                }
                
                else {
                    intakeFL.setPower(0);
                    intakeFR.setPower(0);
                    intakeM.setPower(0);
                    intakeB.setPower(0);
                    latch4 = true;
                    in = false;
                }
                
                test2 = true;
            }
            
            if (test2) {
                latch2 = !latch2;
                test2 = false;
            }
            
            while (gamepad2.right_trigger > 0){
                if (latch4) {
                    intakeFL.setPower(1);
                    intakeFR.setPower(1);
                    intakeM.setPower(.8);
                    intakeB.setPower(.8);
                    latch2 = false;
                    in = false;
                }
                
                else {
                    intakeFL.setPower(0);
                    intakeFR.setPower(0);
                    intakeM.setPower(0);
                    intakeB.setPower(0);
                    latch2 = true;
                    in = false;
                }
                
                test4 = true;
            }
            
            if (test4) {
                latch4 = !latch4;
                test4 = false;
            }
            
            else if (sensorDistance.getDistance(DistanceUnit.CM) < 8 && !latch4 && in) {
                intakeFL.setPower(0);
                intakeFR.setPower(0);
                intakeM.setPower(0);
                intakeB.setPower(0);
                latch2 = true;
                latch4 = true;
                
                in = false;
            }
        }
    }
}
