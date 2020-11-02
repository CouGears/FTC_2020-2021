package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class Test extends OpMode {
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
                robot.motorsFast("go", 1000);
                break;
            case 1:
                robot.motors("stop", 1000);
                break;
        }
        
        telemetry.addData("Case:", robot.counter);
        telemetry.update();
    }
}