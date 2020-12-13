package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import android.graphics.Color;

@TeleOp

public class CompetitionDriving_Copy extends LinearOpMode{
    private boolean serv = false, shoot = false;
    private DcMotor motorBR, motorBL, motorFL, motorFR, intakeFL, shooter, arm, scissorMotor;
    private Servo shooterServo, armServo, frontScissor;
    private DistanceSensor sensorDistance;
    private ColorSensor sensorColor1, sensorColor2;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    int x = 0;
    boolean latch1 = true, test1 = false, latch2 = true, test2 = false, latch3 = true, test3 = false, latch4 = true, test4 = false, in = false, s = false;

    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        intakeFL = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        scissorMotor = hardwareMap.get(DcMotor.class, "scissorMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        frontScissor = hardwareMap.get(Servo.class, "frontScissor");



        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        scissorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setTargetPosition(0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            AutonMethods_Copy competition = new AutonMethods_Copy();
            /*(Color.RGBToHSV((int) (sensorColor.red() * 255),
                (int) (sensorColor.green() * 255),
                (int) (sensorColor.blue() * 255),
                hsvValues);

            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Distance", sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("In", in);
            telemetry.update();
            */
            if(gamepad1.a){
                x = 0;
            }

            else if(gamepad1.b){
                x = 1;
            }

            if(x == 0){
                motorFL.setPower(((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x)));
                motorBL.setPower(((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x)));
                motorBR.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x)));
                motorFR.setPower(((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x)));
            }

            else if(x == 1){
                motorFL.setPower(((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/4);
                motorBL.setPower(((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/4);
                motorBR.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.right_stick_x))/4);
                motorFR.setPower(((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.right_stick_x))/4);
            }
            if(gamepad2.right_bumper) {
                shooter.setPower(1);
            }else{
                shooter.setPower(0);
            }
            if(gamepad2.x) {
                intakeFL.setPower(1);
            } else if(gamepad2.y){
                intakeFL.setPower(-1);
            } else {
                intakeFL.setPower(0);
            }

            if(gamepad2.a && s == false) {
                scissorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontScissor.setPosition(.4);
                competition.sleep(150);
                while(scissorMotor.GetCurrentPosition() < 430){
                    scissorMotor.setPower(1);
                }
                scissorMotor.setPower(0);
                telemetry.addData("Status", "x");

                telemetry.update();
                s = true;
            }
            else if(gamepad2.b && s == true) {
                scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontScissor.setPosition(.1);
                scissorMotor.setTargetPosition(0);
                scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                scissorMotor.setPower(1);
                telemetry.addData("Status", "y");
                telemetry.update();
                s = false;
            }
            /*if(gamepad2.a){
            shooter.setPower(0);
            intakeFL.setPower(0);
            }*/
            if(gamepad2.dpad_right){
                serv = !serv;
                try {
                    Thread.sleep(250);
                }
                catch (InterruptedException e) {
                }
            }
            if(gamepad2.dpad_down){
                shooterServo.setPosition(.6);
            }
            else if(gamepad2.dpad_up){
                shooterServo.setPosition(.8);
            }
            if (serv == false){
                armServo.setPosition(.5);
            }else {
                armServo.setPosition(1); }
            //  if (shoot == false){
            //     shooterServo.setPosition(.2);
            // }else {
            //     shooterServo.setPosition(1);
            // }



            arm.setPower(.25*gamepad2.right_stick_y);
        }
    }
}
