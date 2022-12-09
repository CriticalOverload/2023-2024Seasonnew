package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private DcMotor motorFR,motorBR,motorFL,motorBL,slide;
    private Servo claw;
    private RobotClass2 robot;
    private BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");

        slide = hardwareMap.dcMotor.get("LS");
        claw = hardwareMap.servo.get("claw");
        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, slide, claw, imu, this, false);
        robot.setupRobot();

        double powerMod = 1;
        telemetry.addData("setup?", true);
        telemetry.update();

        
        int MOTOR_TICK_COUNTS = 384;
        waitForStart();
        while(opModeIsActive()){

            //slides
            if(gamepad1.a){
                //set slide
            }
            if(gamepad1.b){
                //set slide
            }
            if(gamepad1.x){
                //set slide
            }
            slide.setPower(gamepad2.right_stick_y);//manual adjustment

            //claw
            if(gamepad2.left_bumper){
                robot.openClaw();
            }
            if(gamepad2.right_bumper){
                robot.closeClaw();
            }

            //driving
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + 3*(Math.PI/4);//-(0)-45=45
           
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
            slide.setPower(-gamepad2.right_stick_y * 0.5);


            //Claw
            if (gamepad2.dpad_up) {
                claw.setPosition(0.5);
            }
            else if (gamepad2.dpad_down) {
                claw.setPosition(1.0);
            }

            //Linear Slide
            if (gamepad2.a) {
                slide.getCurrentPosition();
                slide.setTargetPosition((int)groundRN);
                slide.setPower(.5); /*figure out what value is best for this*/
            }
            else if (gamepad2.x) {
                slide.getCurrentPosition();
                slide.setTargetPosition((int)smallRN);
                slide.setPower(.5); /*figure out what value is best for this*/
            }
            else if (gamepad2.b) {
                slide.getCurrentPosition();
                slide.setTargetPosition((int)medRN);
                slide.setPower(.5); /*figure out what value is best for this*/
            }
            else if (gamepad2.y) {
                slide.getCurrentPosition();
                slide.setTargetPosition((int)highEP);
                slide.setPower(.5); /*figure out what value is best for this*/
            }
            else {
                slide.setPower(0);

            }


            telemetry.addData("Slide position",slide.getCurrentPosition());
            telemetry.update();

        }
    }
}
