package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="auto testing distances")
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, imu, this, false);
        robot.setupRobot_base();
        waitForStart();
        robot.gyroStrafeEncoder(0.5,0,24);
        Thread.sleep(2000);
        robot.gyroStrafeEncoder(0.5,90,24);
        Thread.sleep(2000);
        robot.gyroStrafeEncoder(0.5,180,24);
        Thread.sleep(2000);
        robot.gyroStrafeEncoder(0.5,-90,24);
        Thread.sleep(2000);
    }
}
