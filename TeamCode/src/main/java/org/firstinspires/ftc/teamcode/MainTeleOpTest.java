package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "AA Main TeleOp test")
public class MainTeleOpTest extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight, motorLS;
    private Servo servo;
    private TouchSensor touch;

    private double powerMod = 0.8;
    private double slidePMod = 1.0;
    private RobotClass2 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        servo = hardwareMap.servo.get("claw");
        motorLS = hardwareMap.dcMotor.get("LS");
        touch = hardwareMap.get(TouchSensor.class,"touch");
        
        robot = new RobotClass2(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorLS, servo, null, this, false);

        robot.setupRobot_base_slide_claw_noimu();
        
        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.right_bumper) {
                powerMod = 0.5;
                telemetry.addData("right bumper","gamepad1");
            }else if(gamepad1.left_bumper){
                telemetry.addData("left bumper","gamepad1");
                powerMod = 0.4;
            }else{
                powerMod = 0.8;
            }

            if(gamepad2.right_bumper) {
                slidePMod = 0.85;
                telemetry.addData("right bumper","gamepad2");
            }else if(gamepad2.left_bumper){
                slidePMod = 0.35;
                telemetry.addData("left bumper","gamepad2");
            }else{
                slidePMod = 1.0;
            }

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);
            
            motorLS.setPower(gamepad2.right_stick_y * slidePMod);
            if(gamepad2.y){
                robot.moveSlides('h',slidePMod);
            }
            if(gamepad2.x){
                robot.moveSlides('m',slidePMod);
            }
            if(gamepad2.b){
                robot.moveSlides('l',slidePMod);
            }
            if(gamepad2.a){
                robot.moveSlides('g',slidePMod);
            }

            //Claw
            if (gamepad2.dpad_up) {
                servo.setPosition(0.5);
            }
            else if (gamepad2.dpad_down) {
                servo.setPosition(0.0);
            }
            telemetry.addData("Slide position",motorLS.getCurrentPosition());
            telemetry.update();

        }


    }
}
