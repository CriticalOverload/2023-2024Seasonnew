package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AA Basic Red Terminal")
public class Auto_BasicRedTerminal extends LinearOpMode {
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
        signal = 3;
        claw = hardwareMap.servo.get("claw");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, slides, claw, imu, this, false);
        robot.setupRobot();

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
        signal = mainPipeline.getSignal();
        telemetry.addData("signal",signal);
        telemetry.update();
        waitForStart();
        //if webcam on the back, then start facing the back. orientation of initial robot only matters up till #2
        //1. read signal

        if(signal==0)
            telemetry.addData("assuming",3);
        telemetry.update();
        //todo: test and update
        //also roadrunner...

        //2. drop in terminal
        //turn ccw 90
        // go forward a square
        //drop the cone
        robot.gyroStrafeEncoder(0.5,-90,2);//moving from the wall a bit
        robot.dropInTerminal(0.5, false);//see robot class for method, should be mirror for red
//        robot.moveSlides(2,0.3);
        robot.gyroStrafeEncoder(0.5,-90,4);


//        robot.moveSlides(0,0.3);
//        //3. turn and go to cone stack and align vertically
//        robot.closeClaw();
//        robot.gyroStrafeEncoder(0.5,-65,40);//backwards... may make it a slight to the rightish to avoid knocking into stack or another robot
//        robot.gyroTurn(90,0.5);
//        robot.moveSlides(4, 0.5);
//        robot.openClaw();

            switch(signal){
                case 1:
                    robot.gyroStrafeEncoder(0.5,-90,20);
                    break;
                case 2:
                    robot.gyroStrafeEncoder(0.5,-90,20);
                    robot.gyroStrafeEncoder(0.5,180,24);
                    //change strafing angle so that it adjusts to the tilt
                    //move somehow
                    break;
//            default:
                case 3:
                    robot.gyroStrafeEncoder(0.5,-90,20);
                    robot.gyroStrafeEncoder(0.5,180,48);
                    //move
                    break;
            }


    }
}
