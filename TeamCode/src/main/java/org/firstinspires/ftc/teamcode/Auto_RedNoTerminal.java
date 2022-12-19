package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto Red No Terminal")
public class Auto_RedNoTerminal extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR;
    private DcMotor slides;
    private Servo claw;

    private BNO055IMU imu;

    private RobotClass2 robot;

    private OpenCvCamera cam;// webcam
    private int width = 640;
    private int height = 480;
    private CVClass mainPipeline;

    private int signal;

    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        slides = hardwareMap.dcMotor.get("LS");
        claw = hardwareMap.servo.get("claw");
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, slides, claw, imu, this, false);
        robot.setupRobot_base();

        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
        mainPipeline = new CVClass();//create new pipeline

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
            @Override
            public void onOpened() {
                cam.setPipeline(mainPipeline);//set webcam pipeline
                cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error...", ":(");
                telemetry.update();
                System.exit(0);
            }
        });

        waitForStart();
        //if webcam on the back, then start facing the back. orientation of initial robot only matters up till #2
        //1. read signal
        signal = mainPipeline.getSignal();
        telemetry.addData("signal",signal);
        if(signal==0)
            telemetry.addData("assuming",3);
        telemetry.update();
        //todo: test and update
        //also roadrunner...
        //2. drop in terminal
        //turn ccw 90
        // go forward a square
        //drop the cone
        robot.gyroTurn(180,0.5);
        telemetry.addData("finished","180 turn");
        telemetry.update();
        Thread.sleep(500);
        //3. turn and go to cone stack and align vertically
        robot.gyroStrafeEncoder(0.5,90,12);
        telemetry.addData("finished","strafe forward 12 in");
        telemetry.update();
        Thread.sleep(500);
        robot.gyroTurn(90,0.5);
        telemetry.addData("finished","90 turn");
        telemetry.update();
        Thread.sleep(500);
        robot.gyroStrafeEncoder(0.5,90,0.5);
        telemetry.addData("finished","0.5 forward");
        telemetry.update();
        Thread.sleep(500);
        // robot.moveSlides(1,0.5);
        robot.openClaw();
        telemetry.addData("finished","claw opened");
        telemetry.update();
        Thread.sleep(500);
        robot.gyroStrafeEncoder(0.5,-90,0.5);
        telemetry.addData("finished","-90 strafe back");
        telemetry.update();
        Thread.sleep(500);
        robot.gyroTurn(-90,0.5);
        robot.gyroStrafeEncoder(0.5,90,18);
        // robot.moveSlides(2,0.5);
        robot.gyroTurn(-90,0.5);
        robot.gyroStrafeEncoder(0.5,90,18);
        robot.pickUp(0.5);
        robot.gyroStrafeEncoder(0.5,-90,18);
        robot.gyroTurn(90,0.5);
        robot.gyroStrafeEncoder(0.5,-90,0.5);
        // robot.moveSlides(2,0.5);
        robot.openClaw();
        //use distance sensor to get to wall...
        //robot.driveToWall... copy from archaic then edit
        //4. pick up cone

        //5. place depending on signal
        }
    }
