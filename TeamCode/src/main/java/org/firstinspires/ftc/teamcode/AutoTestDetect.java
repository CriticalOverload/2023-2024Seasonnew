package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Auto test detect")
public class AutoTestDetect extends LinearOpMode {
    //for OpenCV
    OpenCvCamera cam;// webcam
    int width = 640;
    int height = 480;
    CVClassLeft mainPipeline;
    CVClassRight mainPipelineR;
    boolean left = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //TODO Only proceed if you have done PIDCalibration and copied kp and kd values!!!!!!! if you are here and you haven't you be doing something wrong
        //setup robot parts

        //setup camera, turn it on
        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
        mainPipeline = new CVClassLeft();//create new pipeline
        mainPipelineR = new CVClassRight();//create new pipeline

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
            @Override
            public void onOpened() {
                cam.setPipeline(mainPipeline);//set webcam pipeline
                cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error...", ":(");
                System.exit(0);
            }
        });

        waitForStart();//if there is a camera error... and it crashes the program... then we need to find a way to "pause stream"
    	int code;
    	int xpos;
        while(opModeIsActive()){
            if(gamepad1.x){
                left= false;
                cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
                    @Override
                    public void onOpened() {
                        cam.setPipeline(mainPipelineR);//set webcam pipeline
                        cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
                    }

                    @Override
                    public void onError(int errorCode) {
                        telemetry.addData("Camera Error...", ":(");
                        System.exit(0);
                    }
                });
                Thread.sleep(1000);
            }
            if(gamepad1.y){
                left= true;
                cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
                    @Override
                    public void onOpened() {
                        cam.setPipeline(mainPipeline);//set webcam pipeline
                        cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
                    }

                    @Override
                    public void onError(int errorCode) {
                        telemetry.addData("Camera Error...", ":(");
                        System.exit(0);
                    }
                });
                Thread.sleep(1000);
            }
            code = mainPipeline.getCode();//get the code before we move
            xpos = mainPipeline.getX();

            telemetry.addData("barcode value", code);
            telemetry.addData("x positions", xpos);
        
            if (code == 0) {
                telemetry.addData("assuming", "1");
            }
            telemetry.update();
        }
    }
}
