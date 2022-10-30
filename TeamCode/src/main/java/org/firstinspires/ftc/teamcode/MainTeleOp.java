package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private DcMotor motorFR,motorBR,motorFL,motorBL, motorLS;
    private RobotClass2 robot;
    private Servo servo;
    double servoPosition = 1.0;
    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorLS = hardwareMap.dcMotor.get("LS");
        servo = hardwareMap.servo.get("claw");
        servo.setPosition(servoPosition);


        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, motorLS, this);
        robot.setupRobot();

        double powerMod = 1;
        telemetry.addData("setup?",true);
        telemetry.update();

        int lSCurrentPosition = motorLS.getCurrentPosition();
        telemetry.addData("Encoder Position", lSCurrentPosition);

        int MOTOR_TICK_COUNTS = 384;
        waitForStart();
        while(opModeIsActive()){
            //right+left-
            //up-down+?!<<<<<<<<<<
            //drive
//            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + 3*(Math.PI/4);//-(0)-45=45
//            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
//            double rotation = gamepad1.left_stick_x;
//
//            double leftPower = r*Math.sin(angle);//1
//            double rightPower = r*Math.cos(angle);//-1

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);

            motorFL.setPower((powerOne - (rotation))*powerMod);
            motorFR.setPower((powerTwo + (rotation))*powerMod);
            motorBL.setPower((powerTwo - (rotation))*powerMod);
            motorBR.setPower((powerOne + (rotation))*powerMod);

            double circumference = 4.409;//circumference of pulley for hub
            double groundRN = 2/circumference; // rotations needed to reach point from 0
            double smallRN =16/circumference;
            double medRN =26/circumference;
            double highEP =37/circumference;// (inches from ground / circumference)* ticks


//            motorFL.setPower((leftPower - (rotation))*powerMod);
//            motorFR.setPower((rightPower + (rotation))*powerMod);
//            motorBL.setPower((rightPower - (rotation))*powerMod);
//            motorBR.setPower((leftPower + (rotation))*powerMod);
            motorLS.setPower(-gamepad2.right_stick_y * 0.5);


            //Claw
            if (gamepad2.dpad_up) {
                servo.setPosition(0.5);
            }
            else if (gamepad2.dpad_down) {
                servo.setPosition(1.0);
            }

            //Linear Slide
            if (gamepad2.a) {
                motorLS.getCurrentPosition();
                motorLS.setTargetPosition((int)groundRN);
                motorLS.setPower(.5); /*figure out what value is best for this*/
            }
            else if (gamepad2.x) {
                motorLS.getCurrentPosition();
                motorLS.setTargetPosition((int)smallRN);
                motorLS.setPower(.5); /*figure out what value is best for this*/
            }
            else if (gamepad2.b) {
                motorLS.getCurrentPosition();
                motorLS.setTargetPosition((int)medRN);
                motorLS.setPower(.5); /*figure out what value is best for this*/
            }
            else if (gamepad2.y) {
                motorLS.getCurrentPosition();
                motorLS.setTargetPosition((int)highEP);
                motorLS.setPower(.5); /*figure out what value is best for this*/
            }
            else {
                motorLS.setPower(0);

            }


        }
    }
}
