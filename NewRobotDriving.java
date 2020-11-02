package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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


@TeleOp

public class NewRobotDriving extends LinearOpMode{
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFR;
    private DcMotor armAngle;
    private DcMotor armSlide;
    private Servo servoLS;
    private Servo servoRS;
    private Servo servoLR;
    private Servo servoRR;
    private CRServo intakeL;
    private CRServo intakeR;
    private Servo angleL;
    private Servo angleR;
    private Servo export;
    BNO055IMU imu;
    int x = 0;
    boolean latch1 = true;
    boolean test1 = false;
    boolean latch2 = true;
    boolean test2 = false;
    boolean latch3 = true;
    boolean test3 = false;
    boolean latch4 = true;
    boolean test4 = false;
    
    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        armAngle = hardwareMap.get(DcMotor.class, "armAngle");
        armSlide = hardwareMap.get(DcMotor.class, "armSlide");
        servoLS = hardwareMap.get(Servo.class, "servoLS");
        servoRS = hardwareMap.get(Servo.class, "servoRS");
        servoLR = hardwareMap.get(Servo.class, "servoLR");
        servoRR = hardwareMap.get(Servo.class, "servoRR");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        angleL = hardwareMap.get(Servo.class, "angleL");
        angleR = hardwareMap.get(Servo.class, "angleR");
        export = hardwareMap.get(Servo.class, "export");
        
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //servoRS.setDirection(FORWARD);
        //servolS.setDirection(FORWARD);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        angleL.setPosition(.7);
        angleR.setPosition(.3);
        servoLR.setPosition(1);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            
            telemetry.addData("intakeL", intakeL.getPower() );
            telemetry.addData("intakeR", intakeR.getPower() );
            telemetry.update();
            
            if(gamepad1.a){
                x = 0;
            }
            
            else if(gamepad1.b){
                x = 1;
            }
            
            else if(gamepad1.x){
                x = 2;
            }
            
            else if(gamepad1.y){
                x = 3;
            }
            
            if(x == 0){ 
                motorFL.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x)));
                motorBL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x)));
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x)));
                motorFR.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x)));
            }
            
            else if(x == 1){ 
                motorFL.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/4);
                motorBL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/4);
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/4);
                motorFR.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/4);
            }
            
            else if(x == 2){ 
                motorFL.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/8);
                motorBL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/8);
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/8);
                motorFR.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/8);
            }
            
            else if(x == 3){ 
                motorFL.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/16);
                motorBL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/16);
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/16);
                motorFR.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/16);
            }
            
            if (gamepad2.a) {
                servoLS.setPosition(0);
                servoRS.setPosition(1);
            }
            
            else {
                servoLS.setPosition(.6);
                servoRS.setPosition(.4);
            }
            
            while (gamepad2.x) {
                if (latch3) {
                    servoRR.setPosition(1);
                }
                
                else {
                    servoRR.setPosition(.8);
                }
                
                test3 = true;
            }
            
            if (test3) {
                latch3 = !latch3;
                test3 = false;
            }
            
            while (gamepad2.y) {
                if (latch1) {
                    servoLR.setPosition(.5);
                }
                
                else {
                    servoLR.setPosition(1);
                }
                
                test1 = true;
            }
            
            if (test1) {
                latch1 = !latch1;
                test1 = false;
            }
            
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double heading = (angles.firstAngle)%360;
            
            armAngle.setPower(-gamepad2.left_stick_y/5);
            armSlide.setPower(-gamepad2.right_stick_y/10);
            
            while (gamepad2.left_trigger > 0) {
                if (latch2) {
                    angleL.setPosition(.2);
                    angleR.setPosition(.8);
                    intakeL.setPower(.8);
                    intakeR.setPower(-.8);
                    servoRR.setPosition(.8);
                    latch3 = true;
                    latch4 = false;
                }
                
                else {
                    angleL.setPosition(.4);
                    angleR.setPosition(.6);
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                    latch4 = true;
                    
                }
                
                test2 = true;
            }
            
            if (test2) {
                latch2 = !latch2;
                test2 = false;
            }
            
            while (gamepad2.right_trigger > 0){
                if (latch4) {
                    angleL.setPosition(.2);
                    angleR.setPosition(.8);
                    intakeL.setPower(-.8);
                    intakeR.setPower(.8);
                    latch2 = false;
                }
                
                else {
                    angleL.setPosition(.4);
                    angleR.setPosition(.6);
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                    latch2 = true;
                }
                
                test4 = true;
            }
            
            if (test4) {
                latch4 = !latch4;
                test4 = false;
            }
            
            if (gamepad2.left_bumper) {
                export.setPosition(.5);
            }
            
            else {
                export.setPosition(1);
            }
        }
    }
}
