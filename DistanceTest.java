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

public class DistanceTest extends LinearOpMode{
    private DistanceSensor bottomSensor, topSensor;
    @Override
    public void runOpMode() {
        bottomSensor = hardwareMap.get(DistanceSensor.class, "bottomSensor");
        topSensor = hardwareMap.get(DistanceSensor.class, "topSensor");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("Top Distance", topSensor.getDistance(DistanceUnit.CM));
//             telemetry.addData("Bottom Distance", bottomSensor.getDistance(DistanceUnit.CM));
//            telemetry.update();
            if (bottomSensor.getDistance(DistanceUnit.CM) < 10 && topSensor.getDistance(DistanceUnit.CM) > 10) {
                telemetry.addData("One ring", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            else if(bottomSensor.getDistance(DistanceUnit.CM) < 5 && topSensor.getDistance(DistanceUnit.CM) < 5) {
                telemetry.addData("Four rings", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            else {
                telemetry.addData("No rings", topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom:", bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
    }
}
