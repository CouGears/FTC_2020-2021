package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class BlueFoundation_OuterPark extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();
    
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
        robot.marker();
        switch (robot.counter) {
            case 0:
                robot.motorsFast("right", 700);
                break;
            case 1:
                robot.sleep(100);
                break;
            case 2:
                robot.motorsFast("back", 2200);
                break;
            case 3:
                robot.servoClamp();
                break;
            case 4:
                robot.sleep(100);
                break;
            case 5:
                robot.motorsFast("turn_left", 400);
                break;
            case 6:
                robot.sleep(100);
                break;
            case 7:
                robot.motorsFast("front", 2300);
                break;
            case 8:
                robot.sleep(100);
                break;
            case 9:
                robot.motors("turn_left", 2000);
                break;
            case 10:
                robot.servoClamp();
                break;
            case 11:
                robot.sleep(100);
                break;
            case 12:
                robot.motorsFast("back", 1700);
                break;
            case 13:
                robot.sleep(100);
                break;
            case 14:
                robot.motorsFast("right", 2000);
                break;
            case 15:
                robot.sleep(100);
                break;
            case 16:
                robot.motorsFast("front", 2000);
                break;
            case 17:
                robot.motors("stop", 1000);
                break;
        }
        
        telemetry.addData("Case:", robot.counter);
        telemetry.update();
    }
}
