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

public class BlueLeft extends OpMode{
    double rev = 383.6;
    double inch = 2*rev/(3.78*3.14);
    double feet = inch*12;
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
            robot.drive(3*feet+6*inch, 5*feet, .6);
            break;
       case 1:
            robot.shootServ(1);
            break;
        case 2:
            robot.shoot(600, .7, .7);
            break;
        case 3:
        	robot.sleep(100);
        	break;
        case 4:
        	robot.drive(-5*inch, 0, .6);
        	break;
        case 5:
        	robot.shoot(600, 0, 1);
        	break;
        case 6:
        	robot.intake(2000);
        	break;
        case 7:
        	robot.drive(-5*inch,0, .6);
        	break;
        case 7:
        	robot.shootServ(.7);
        	break;
        case 8:
        	robot.shoot(600, 0, 1);
        	break;
    }
        
        telemetry.addData("Case:", robot.counter);
        telemetry.update();
    }
}
