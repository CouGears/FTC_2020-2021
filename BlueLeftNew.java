package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

public class BlueLeftNew extends OpMode {
    double rev = 383.6;
    double inch = 2 * rev / (3.78 * 3.14);
    double feet = inch * 12;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods_Copy robot = new AutonMethods_Copy();
    int diamond = 0;
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
        int rings = 0;
        switch (robot.counter) {

            // case 0:
            //     robot.drive(6*feet,0*feet, 1);
            //     robot.counter++;
            //     break;
            // case 1:
            //     robot.shoot(true);
            //     robot.counter++;
            //     break;
            // case 2:
            //     runtime.reset();
            //     while(runtime.seconds()<5);
            //     robot.counter++;
            //     break;
            // case 3:
            //     robot.shoot(false);
            //     robot.counter++;
            //     break;

            case 0:
                robot.drive(1*feet,1*feet,1);
                robot.counter++;
                break;
            case 1:
                diamond = robot.distance();
                robot.counter++;
                break;
            case 2:
                robot.drive(3*feet, -1*feet,1);
                robot.counter++;
                break;
            case 3:
                robot.shoot(true);
                robot.counter++;
                break;
            case 4:
                runtime.reset();
                while(runtime.seconds()<2);
                robot.counter++;
                break;
            case 5:
                robot.shootServ(.8);
                robot.counter++;
                break;
            case 6:
                runtime.reset();
                while(runtime.seconds()<1);
                robot.counter++;
                break;
            case 7:
                robot.shootServ(0);
                robot.counter++;
                break;
            case 8:
                robot.drive(0,5*inch, 1);
                robot.counter++;
                break;
            case 9:
                robot.shootServ(.8);
                robot.counter++;
                break;
            case 10:
                runtime.reset();
                while(runtime.seconds()<1);
                robot.counter++;
                break;
            case 11:
                robot.shootServ(0);
                robot.counter++;
                break;

            case 12:
                robot.drive(0,5*inch, 1);
                robot.counter++;
                break;

            case 13:
                robot.shootServ(.8);
                robot.counter++;
                break;
            case 14:
                runtime.reset();
                while(runtime.seconds()<1);
                robot.counter++;
                break;
            case 15:
                robot.shootServ(0);
                robot.counter++;
                break;

            case 16:
                robot.drive(0,5*inch, 1);
                robot.counter++;
                break;

            case 17:
                robot.shootServ(.8);
                robot.counter++;
                break;
            case 18:
                runtime.reset();
                while(runtime.seconds()<1);
                robot.counter++;
                break;
            case 19:
                robot.shootServ(0);
                robot.counter++;
                break;
            case 20:
                robot.shoot(false);
                robot.counter++;
                break;
            case 21:
                robot.drive(0,5*inch, 1);
                robot.counter++;
                break;
            case 22:
                if(diamond == 1){
                    robot.drive(0,-1*feet,1);
                }
                else if(diamond == 4){
                    robot.drive(2*feet,0,1);
                }
                else if(diamond == 0){
                    robot.drive(3*feet,-3*feet,1);
                }
                robot.counter++;
                break;
        }
    }
}