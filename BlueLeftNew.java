package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class BlueLeftNew extends OpMode {
    double rev = 383.6;
    double inch = 2 * rev / (3.78 * 3.14);
    double feet = inch * 12;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods_Copy robot = new AutonMethods_Copy();

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        switch (robot.counter) {
            case 0:
                robot.drive(2 * feet, 2 * feet, 5);
                robot.counter++;
                break;
            case 1:
                robot.drive(2 * feet, 0, 5);
                break;
            case 2:
                robot.shoot();
                break;
            case 3:
                int rings = robot.distance();
                if (rings == 4) {
                    robot.drive(-1 * feet, 4 * feet, 5);
                } else if (rings == 1) {
                    robot.drive(o, 3 * feet, 5);
                } else {
                    robot.drive(-1 * feet, 1 * feet, 5);
                }
                break;


                telemetry.addData("Case:", robot.counter);
                telemetry.update();
        }
    }
