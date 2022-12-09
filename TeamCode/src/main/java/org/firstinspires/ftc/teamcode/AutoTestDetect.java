package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Autonomous(name = "Auto test detect")
@Config
public class AutoTestDetect extends LinearOpMode {
    //for OpenCV
    OpenCvCamera cam;// webcam
    int width = 640;
    int height = 480;
    CVClass mainPipeline;
    CVClassTry2 try2;
    
//    OpenCvInternalCamera2 phonecam;

    @Override
    public void runOpMode() throws InterruptedException {
        //setup camera, turn it on
        //for previewing on app
        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //getting live preview and phone camera
//        phonecam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, camViewID);
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
        mainPipeline = new CVClass();//create new pipeline
        try2 = new CVClassTry2();

        RobotClass2 dashboarder = new RobotClass2(this);
        dashboarder.setupDashboard();

        /*phonecam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
            @Override
            public void onOpened() {
                phonecam.setPipeline(mainPipeline);//set webcam pipeline
                phonecam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
            }

            @Override
            public void onError(int errorCode) {
                dashboarder.addData("Camera Error...", ":(");
                dashboarder.update();
                System.exit(0);
            }
        });*/

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
            @Override
            public void onOpened() {
                cam.setPipeline(mainPipeline);//set webcam pipeline
                cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
            }

            @Override
            public void onError(int errorCode) {
                dashboarder.addData("Camera Error...", ":(");
                dashboarder.update();
                System.exit(0);
            }
        });
        boolean main = true;
        dashboarder.addData("please work","this time");
        dashboarder.update();
        waitForStart();//if there is a camera error... and it crashes the program... then we need to find a way to "pause stream"
    	int signal=0;
        while(opModeIsActive()){
            dashboarder.addData("yay!","first!");
            dashboarder.update();
            telemetry.addData("Got to this point","yay");
            telemetry.update();
            Thread.sleep(1000);
            /*if(gamepad1.a){
                phonecam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
                    @Override
                    public void onOpened() {
                        phonecam.setPipeline(mainPipeline);//set webcam pipeline
                        phonecam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
                    }

                    @Override
                    public void onError(int errorCode) {
                        dashboarder.addData("Camera Error...", ":(");
                        dashboarder.update();
                        System.exit(0);
                    }
                });

                *//*cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
                    @Override
                    public void onOpened() {
                        cam.setPipeline(mainPipeline);//set webcam pipeline
                        cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
                    }

                    @Override
                    public void onError(int errorCode) {
                        dashboarder.addData("Camera Error...", ":(");
                        dashboarder.update();
                        System.exit(0);
                    }
                });*//*
                main = true;
            }
            if(gamepad1.b){
                phonecam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
                    @Override
                    public void onOpened() {
                        phonecam.setPipeline(try2);//set webcam pipeline
                        phonecam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
                    }

                    @Override
                    public void onError(int errorCode) {
                        dashboarder.addData("Camera Error...", ":(");
                        dashboarder.update();
                        System.exit(0);
                    }
                });

                *//*cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
                    @Override
                    public void onOpened() {
                        cam.setPipeline(try2);//set webcam pipeline
                        cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
                    }

                    @Override
                    public void onError(int errorCode) {
                        dashboarder.addData("Camera Error...", ":(");
                        dashboarder.update();
                        System.exit(0);
                    }
                });*//*
                main = false;
            }*/
//            if(main){
                signal = mainPipeline.getSignal();
                dashboarder.addData("black box height", mainPipeline.getHeight());
//            }
            /*else{
                signal = try2.getSignal();
                dashboarder.addData("black box height", try2.getHeight());
            }*/

            telemetry.addData("signal value", signal);
        
            if (signal == 0) {
                telemetry.addData("assuming", "1");
            }
            telemetry.addData("boxPDiffs", mainPipeline.getBoxPDiffs());
            telemetry.addData("blackContourSize", mainPipeline.getContourSize());
            telemetry.update();
            Thread.sleep(1000);
        }
    }
}
