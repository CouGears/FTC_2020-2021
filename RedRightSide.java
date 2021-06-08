package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
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

public class RedRightSide extends OpMode { //hello there
    double rev = 383.6;
    double inch = rev / (3.78 * 3.14);
    double feet = inch * 12;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();
    int diamond = 0;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
       /* while (!robot.scissorTouch.isPressed()) {
            robot.scissorMotor.setPower(-1);
        }
        robot.scissorMotor.setPower(0);
        while (!robot.armTouch.isPressed()) {
            robot.arm.setPower(1);
        }
        robot.arm.setPower(0);
        scissorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        int rings = 0;
        switch (robot.counter) {
            case 0:
                telemetry.addData("Top Distance", robot.topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom Distance", robot.bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
                //robot.scissorServUp();
                robot.armBlock.setPosition(.5);
                robot.counter++;
                break;
            case 1:
                robot.armServ(.9);
                robot.counter++;
                break;
            case 2:
                robot.driveWithDecel(3.2*feet, -12*inch);
                robot.counter++;
                break;
            case 3:
                robot.scissorServUp();
                robot.counter++;
                break;
            case 4:
                robot.newSleep(1);
                robot.counter++;
                break;
            case 5:
                diamond = robot.distance();
                telemetry.addData("rings:", diamond);
                telemetry.addData("Top Distance", robot.topSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Bottom Distance", robot.bottomSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
                robot.turn(-5.0);
                robot.sleep(200);
                robot.counter++;
                break;
            case 6:
                robot.shoot(-.6);
                robot.arm(-1800);
                robot.sleep(2000);
                robot.counter++;
                break;
            case 7:
                robot.topGoal();
                robot.counter++;
                break;
            case 8:
                robot.topGoal();
                robot.counter++;
                break;
            case 9:
                robot.topGoal();
                robot.counter++;
                break;
            case 10:
                robot.shoot(0);
                robot.arm(0);
                robot.turn(5.0);
                robot.counter++;
                break;
            case 11:
                robot.counter++;
                break;
            case 12:
                if (diamond == 0) {
                    robot.driveWithDecel(2.1*feet, 1.2*feet);
                } else if (diamond == 1) {
                    robot.driveWithDecel(3 * feet, -1 * feet);
                } else if (diamond == 4) {
                    robot.driveWithDecel(5.6 * feet, 1.2 * feet);
                }
                robot.counter++;
                break;
            case 13:
                // robot.armTime(.45,-1);
                robot.arm(-1800);
                robot.sleep(500);
                robot.counter++;
                break;
            case 14:
                robot.armServ(0.5);
                robot.counter++;
                break;
            case 15:
                robot.scissorServDown();
                robot.counter++;
                break;
            case 16:
                if (diamond == 0) {
                    robot.drive(-.5*feet, 0,0.3);
                } else if (diamond == 1) {
                    robot.drive(0, 0 * feet,0.3);
                } else if (diamond == 4) {
                    robot.drive(-3 * feet, -.2 * feet,0.3);
                }
                robot.counter++;
                break;
            case 17:
                robot.scissorServDown();
                robot.counter++;
                break;
            case 18:
                robot.armBlock.setPosition(.55);
                robot.counter++;
                break;
            case 19:
                robot.armServ(.8);
                robot.counter++;
                break;
        }
    }
}
