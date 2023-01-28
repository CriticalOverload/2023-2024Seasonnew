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

@Autonomous(name="AA Basic Red (Left Auto)")
public class Auto_BasicRedTerminal extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR;
    private DcMotor slides;
    private Servo claw;

    private BNO055IMU imu;

    private RobotClass2 robot;

    private OpenCvCamera cam;// webcam
    private int width = 640;
    private int height = 480;
    private CVClass2 mainPipeline;

    private int signal;

    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        slides = hardwareMap.dcMotor.get("LS");
        claw = hardwareMap.servo.get("claw");
//        signal = 3;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new RobotClass2(motorFL, motorFR, motorBL, motorBR, slides, claw, imu, this, false);
        robot.setupRobot();

        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
        mainPipeline = new CVClass2();//create new pipeline

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
        while(!opModeIsActive()){
            signal = mainPipeline.getSignal();
            telemetry.addData("signal", signal);
            telemetry.update();
        }//but why
        waitForStart();
        //if webcam on the back, then start facing the back. orientation of initial robot only matters up till #2
        //1. read signal
        signal = mainPipeline.getSignal();
        telemetry.addData("signal",signal);
        telemetry.update();
        if(signal==0)
            telemetry.addData("assuming",3);
        telemetry.update();

//        robot.gyroStrafeEncoder(0.5,-90,2);//moving from the wall a bit
        robot.dropInTerminal(0.5, false);//see robot class for method, should be mirror for red
        switch(signal){
            case 1:
                robot.gyroStrafeEncoder(0.5,180,25);
                robot.gyroStrafeEncoder(0.5,-90,30);
                robot.gyroStrafeEncoder(0.5,0,25);
                break;
            case 2:
                robot.gyroStrafeEncoder(0.5,180,28);
                robot.gyroStrafeEncoder(0.5,-90,30);
                //change strafing angle so that it adjusts to the tilt
                //move somehow
                break;
            case 3:
            default:
                robot.gyroStrafeEncoder(0.5,180,54);
                robot.gyroStrafeEncoder(0.5,-90,30);
                //move
                break;
        }


    }
}
