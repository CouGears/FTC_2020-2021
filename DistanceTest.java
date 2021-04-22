package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Locale;
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

public class DistanceTest extends LinearOpMode{
    private DistanceSensor bottomSensor, topSensor, sensorDistance;
    private ColorSensor sensorColor;
    @Override
    public void runOpMode() {
        bottomSensor = hardwareMap.get(DistanceSensor.class, "bottomSensor");
        topSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (bottomSensor.getDistance(DistanceUnit.CM) < 25 && topSensor.getDistance(DistanceUnit.CM) > 4) {
                telemetry.addData("One ring", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            else if(bottomSensor.getDistance(DistanceUnit.CM) < 25 && topSensor.getDistance(DistanceUnit.CM) <4 ) {
                telemetry.addData("Four rings", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            else {
                telemetry.addData("No rings", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        /*    Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColopackage org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Locale;
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

public class DistanceTest extends LinearOpMode{
    private DistanceSensor bottomSensor, topSensor, sensorDistance;
    private ColorSensor sensorColor;
    @Override
    public void runOpMode() {
        bottomSensor = hardwareMap.get(DistanceSensor.class, "bottomSensor");
        topSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (bottomSensor.getDistance(DistanceUnit.CM) < 25 && topSensor.getDistance(DistanceUnit.CM) > 4) {
                telemetry.addData("One ring", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            else if(bottomSensor.getDistance(DistanceUnit.CM) < 25 && topSensor.getDistance(DistanceUnit.CM) <4 ) {
                telemetry.addData("Four rings", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            else {
                telemetry.addData("No rings", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        /*    Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();*/
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.update();
        }
    }
}
r.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();*/
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.update();
        }
    }
}
