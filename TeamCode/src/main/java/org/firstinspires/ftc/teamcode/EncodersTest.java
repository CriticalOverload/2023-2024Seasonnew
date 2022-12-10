package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Testing Dead Wheels")
public class EncodersTest extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR;
    private DcMotor leftEncoder, rightEncoder, backEncoder;

    private BNO055IMU imu;

    private RobotClass2 robot;

    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        leftEncoder = hardwareMap.dcMotor.get("encLeft");
        rightEncoder = hardwareMap.dcMotor.get("encRight");
        backEncoder = hardwareMap.dcMotor.get("encBack");

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, leftEncoder, rightEncoder, backEncoder, imu, this, false);
        robot.setupRobot();

        waitForStart();
        robot.gyroStrafeEncoder_deadWheels(0.5, 90, 24);
    }
}
