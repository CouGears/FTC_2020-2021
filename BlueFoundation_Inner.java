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

public class BlueFoundation_Inner extends OpMode {
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
                robot.motors("right", 600);
                break;
            case 1:
                robot.sleep(250);
                break;
            case 2:
                robot.motors("back", 2000);
                break;
            case 3:
                robot.servoClamp();
                break;
            case 4:
                robot.sleep(250);
                break;
            case 5:
                robot.motors("front", 2400);
                break;
            case 6:
                robot.motors("left_turn", 90);
                break;
		case 7:
			robot.servoClamp();
			break;
		case 8:
			robot.sleep(250);
			break;
		case 9:
			robot.motors("back", 1700);
			break;
		case 10:
			robot.motors("stop", 0);
			break;
	}
        
        telemetry.addData("Case:", robot.counter);
        telemetry.update();
    }
}
