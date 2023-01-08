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

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        
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
                robot.moveSlides(0,0.5);
            }
            if(gamepad1.b){
                robot.moveSlides(2,0.5);
            }
            if(gamepad1.x){
                robot.moveSlides(1,0.5);
            }
            if(gamepad1.y){
                robot.moveSlides(3,0.5);
            }

            slide.setPower(-gamepad2.right_stick_y*0.5);

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



//            motorFL.setPower((leftPower - (rotation))*powerMod);
//            motorFR.setPower((rightPower + (rotation))*powerMod);
//            motorBL.setPower((rightPower - (rotation))*powerMod);
//            motorBR.setPower((leftPower + (rotation))*powerMod);

            telemetry.addData("Slide position",slide.getCurrentPosition());
            telemetry.update();

        }
    }
}
