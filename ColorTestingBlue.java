package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class ColorTestingBlue extends OpMode {
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
                robot.motorsFast("front", 1200);
                break;
            case 1:
                robot.sleep(100);
                break;
            case 2:
                robot.checkBlocks("blue");
                break;
            case 3:
                robot.getBlockAndFoundation("blue");
                break;
        }
        
        telemetry.addData("Case:", robot.counter);
        telemetry.addData("Block:", robot.block());
        telemetry.addData("Sensor 1:", robot.color1());
        telemetry.addData("Sensor 2:", robot.color2());
        telemetry.update();
    }
}