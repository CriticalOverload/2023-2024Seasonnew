package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Base Test")
public class BaseTest extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight, motorLS;
    private Servo servo;
    double servoPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        servo = hardwareMap.servo.get("claw");
        motorLS = hardwareMap.dcMotor.get("LS");
        servo.setPosition(servoPosition);

        //reverse the needed motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double powerMod = 0.8;
        double slidePMod = 1.0;
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

            if(gamepad1.dpad_up){
                motorFrontLeft.setPower(powerMod);
                motorFrontRight.setPower(powerMod);
                motorBackLeft.setPower(powerMod);
                motorBackRight.setPower(powerMod);
            }


            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);
//            double powerOne = r*Math.cos(angle);
//            double powerTwo = r*Math.sin(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);
            motorLS.setPower(gamepad2.right_stick_y * 0.6);


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
