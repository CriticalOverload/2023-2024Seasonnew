package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoTesting extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR;
    private BNO055IMU imu;

    private RobotClass2 robot;
    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");

        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, imu, this, false);
        robot.setupRobot();
        waitForStart();
        robot.gyroStrafeEncoder(0.5,0,24);
    }

}
