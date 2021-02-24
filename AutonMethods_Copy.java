package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
import com.qualcomm.robotcore.util.ElapsedTime;
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

public class AutonMethods_Copy {

    //Constructor
    public AutonMethods_Copy() {

    }

    //Declare and initial variables
    double FRtpos, BRtpos, FLtpos, BLtpos;
    public static DcMotor motorBR, motorBL, motorFL, motorFR, intakeFL, shooter, arm, scissorMotor;
    private static Servo shooterServo, armServo, marker, frontScissor, backScissor;
    public static DistanceSensor topSensor, bottomSensor;
    public TouchSensor armTouch, scissorTouch;
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;

    private double speed;
    private boolean clampDown = false;
    public int counter = 0;



    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        intakeFL = map.get(DcMotor.class, "intake");
        arm = map.get(DcMotor.class, "arm");
        shooter = map.get(DcMotor.class, "shooter");
        bottomSensor = map.get(DistanceSensor.class, "bottomSensor");
        topSensor = map.get(DistanceSensor.class, "topSensor");

        armServo = map.get(Servo.class, "armServo");
        shooterServo = map.get(Servo.class, "shooterServo");
        //note - this is according to front orientation - front is in the front and back is in the back
        //also these should be configured accordingly
        scissorMotor = map.get(DcMotor.class, "scissorMotor");


        bottomSensor = map.get(DistanceSensor.class, "bottomSensor");
        topSensor = map.get(DistanceSensor.class, "topSensor");
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scissorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);


        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);

        scissorMotor.setTargetPosition(0);
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

        // tele.addData(">", "Gyro Calibrating. Do Not Move!");
        // tele.update();
    }

    //Function to move the robot in any direction
    public void drive(double forward, double sideways, double spee) {
        runtime.reset();
        while (motorFR.isBusy() || motorFL.isBusy()){
            if(runtime.seconds > 3) break;
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRtpos = forward - sideways;
        BRtpos = forward + sideways;
        FLtpos = forward + sideways;
        BLtpos = forward - sideways;

        motorFL.setTargetPosition((int)FLtpos);
        motorBL.setTargetPosition((int)BLtpos);
        motorFR.setTargetPosition(-(int)FRtpos);
        motorBR.setTargetPosition(-(int)BRtpos);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed(spee);


    }


    public void scissorServUp() {

        sleep(150);
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setTargetPosition(-3300);
        scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scissorMotor.setPower(1);
        sleep(500);

    }

    public void scissorServDown(){
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setTargetPosition(0);
        scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scissorMotor.setPower(1);

        sleep(500);
    }


    public void shootServ(double pos) {
        while (motorFR.isBusy() || motorFL.isBusy());
        shooterServo.setPosition(pos);


    }

    public void armServ(double pos) {
        while (motorFR.isBusy() || motorFL.isBusy());
        armServo.setPosition(pos);

    }

    public void arm(int pos) {
        while (arm.getCurrentPosition() < pos){
            arm.setPower(1);
        }
        while (arm.getCurrentPosition() > pos){
            arm.setPower(-1);
        }
        arm.setPower(0);

    }

    public void shoot(boolean onOff) {
        while (motorFR.isBusy() || motorFL.isBusy()) ;
        if(onOff){
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }
    }

    public int distance() {
        int rings = 0;
        if (bottomSensor.getDistance(DistanceUnit.CM) < 24 && topSensor.getDistance(DistanceUnit.CM) > 24) {
            // tele.addData("One ring", bottomSensor.getDistance(DistanceUnit.CM));
            // tele.update();
            rings = 1;
        } else if (bottomSensor.getDistance(DistanceUnit.CM) < 24 && bottomSensor.getDistance(DistanceUnit.CM) < 24) {
            // tele.addData("Four rings", topSensor.getDistance(DistanceUnit.CM));
            // tele.update();
            rings = 4;
        } else {
            // tele.addData("No rings", bottomSensor.getDistance(DistanceUnit.CM));
            // tele.update();
            rings = 0;
        }
        return rings;
    }


    public void intake(int time) {
        intakeFL.setPower(1);
        sleep(time);
        intakeFL.setPower(0);

    }



    //Function to have the robot sleep
    public void sleep(long sleep) {
        try {
            Thread.sleep(sleep);
        } catch (InterruptedException e) {
            tele.addLine("Failed Sleep");
            tele.update();
        }


    }

    public void speed (double spee){
        motorFL.setPower(spee);
        motorBL.setPower(spee);
        motorFR.setPower(spee);
        motorBR.setPower(spee);
    }

}
