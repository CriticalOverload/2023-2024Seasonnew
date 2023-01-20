package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//@Autonomous(name = "Camera Testing idk")//great name....
public class CameraTest extends LinearOpMode {
    OpenCvCamera cam;// webcam
    int width = 640;
    int height = 480;
    CVClass2 mainPipeline;

    @Override
    public void runOpMode() throws InterruptedException{
        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());//view for viewing what the webcam sees I think, on ds. Double check :grimacing:
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
        mainPipeline = new CVClass2();//create new pipeline
        
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {        
                cam.setPipeline(mainPipeline);//set webcam pipeline
                cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
            }
            
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error...",":(");
                telemetry.update();
                System.exit(0);
            }
        });

        telemetry.addData("signal:",mainPipeline.getSignal());
        telemetry.update();
        
        waitForStart();
        
        while(opModeIsActive()){
            telemetry.addData("signal:",mainPipeline.getSignal());
            telemetry.addData("boxHeight", mainPipeline.getHeight());
            telemetry.update();
        }
        //now start button is pressed, robot go!
    }

    
}
