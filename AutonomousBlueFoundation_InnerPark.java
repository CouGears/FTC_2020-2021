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

public class AutonomousBlueFoundation_InnerPark extends OpMode {
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
        switch (robot.counter) {
            case 0:
                robot.motors("right", 700);
                break;
            case 1:
                robot.sleep(250);
                break;
            case 2:
                robot.motors("back", 2200);
                break;
            case 3:
                robot.servoClamp();
                break;
            case 4:
                robot.sleep(250);
                break;
            case 5:
                robot.motors("front", 2200);
                break;
            case 6:
                robot.sleep(250);
                break;
            case 7:
                robot.motors("turn_left", 3000);
                break;
            case 8:
                robot.servoClamp();
                break;
            case 9:
                robot.sleep(250);
                break;
            case 10:
                robot.motors("back", 1300);
                break;
            case 11:
                robot.sleep(250);
                break;
            case 12:
                robot.motors("back", 500);
                break;
            case 13:
                robot.sleep(250);
                break;
            case 14:
                robot.motors("front", 2500);
                break;
            case 15:
                robot.motors("stop", 1000);
                break;
        }
        
        telemetry.addData("Case:", robot.counter);
        telemetry.update();
    }
}
