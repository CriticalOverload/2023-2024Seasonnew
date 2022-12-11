package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto test detect")
@Config
public class AutoTestDetect extends LinearOpMode {
    //for OpenCV
    OpenCvCamera cam;// webcam
    int width = 640;
    int height = 480;
    CVClass mainPipeline;
    
//    OpenCvInternalCamera2 phonecam;

    @Override
    public void runOpMode() throws InterruptedException {
        //setup camera, turn it on
        //for previewing on app
        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //getting live preview and phone camera
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
        mainPipeline = new CVClass();//create new pipeline

        RobotClass2 dashboarder = new RobotClass2(this);
        dashboarder.setupDashboard();

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

        dashboarder.addData("please work","this time");
        dashboarder.update();
        waitForStart();//if there is a camera error... and it crashes the program... then we need to find a way to "pause stream"
    	int signal=0;
        while(opModeIsActive()){
            dashboarder.addData("yay!","first!");
            // telemetry.addData("Got to this point","yay");
            // telemetry.update();
            signal = mainPipeline.getSignal();
            dashboarder.addData("black box height", mainPipeline.getHeight());

            dashboarder.addData("signal value", signal);
        
            if (signal == 0) {
                dashboarder.addData("assuming", "1");
            }
            dashboarder.addData("boxPDiffs", mainPipeline.getBoxPDiffs());
            dashboarder.addData("blackContourSize", mainPipeline.getContourSize());
            dashboarder.update();
            Thread.sleep(1000);
        }
    }
}
