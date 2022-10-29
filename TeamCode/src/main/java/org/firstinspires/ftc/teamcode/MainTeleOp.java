package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private DcMotor motorFR,motorBR,motorFL,motorBL;
    private RobotClass2 robot;
    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");

        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, this);
        robot.setupRobot();

        double powerMod = 1;
        telemetry.addData("setup?", true);
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            //right+left-
            //up-down+?!<<<<<<<<<<
            //drive
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + 3*(Math.PI/4);//-(0)-45=45
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double leftPower = r*Math.sin(angle);//1
            double rightPower = r*Math.cos(angle);//-1

            motorFL.setPower((leftPower - (rotation))*powerMod);
            motorFR.setPower((rightPower + (rotation))*powerMod);
            motorBL.setPower((rightPower - (rotation))*powerMod);
            motorBR.setPower((leftPower + (rotation))*powerMod);

        }
    }
}
