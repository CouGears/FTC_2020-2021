package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous

public class Timed extends OpMode {
    private static DcMotor motorFL, motorBL, motorBR, motorFR, intakeFL, intakeFR, intakeM, intakeB;
    private static Servo servoLS, servoRS, clamp, marker;
    private static ColorSensor sensorColor1, sensorColor2;
    private static DistanceSensor sensorDistance;
    HardwareMap map;
    Telemetry tele;
    
    public void init() {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        intakeFL = map.get(DcMotor.class, "intakeFL");
        intakeFR = map.get(DcMotor.class, "intakeFR");
        intakeM = map.get(DcMotor.class, "intakeM");
        intakeB = map.get(DcMotor.class, "intakeB");
        servoLS = map.get(Servo.class, "servoLS");
        servoRS = map.get(Servo.class, "servoRS");
        clamp = map.get(Servo.class, "clamp");
        sensorColor1 = map.get(ColorSensor.class, "sensorColor1");
        sensorColor2 = map.get(ColorSensor.class, "sensorColor2");
        sensorDistance = map.get(DistanceSensor.class, "sensorDistance");
        marker = map.get(Servo.class, "marker");
    }
    
    @Override
    public void loop() {
        try {
            Thread.sleep(29500);
        } catch (InterruptedException e) {}
        
        motorFL.setPower(1);
        motorBL.setPower(1);
        motorFR.setPower(1);
        motorBR.setPower(1);
    }
}