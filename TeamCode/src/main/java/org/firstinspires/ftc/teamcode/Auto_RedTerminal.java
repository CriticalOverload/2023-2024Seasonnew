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

@Autonomous(name="Auto Red Terminal")
public class Auto_RedTerminal extends LinearOpMode {
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
//        robot.dropInTerminal(0.5, blue);
        //3. turn and go to cone stack and align vertically
        robot.gyroStrafeEncoder(0.5,90,48);
        robot.gyroStrafeEncoder(0.5,180,6);
        robot.gyroTurn(-90,0.5);//change!!!
        // robot.moveSlides(1,0.5);
        //use distance sensor to get to wall...
        //robot.driveToWall... copy from archaic then edit
        //4. pick up cone
        robot.pickUp(0.5);
        //5. place depending on signal
        switch(signal){
            case 1:
                //place in high close to audience
                robot.goToHigh(0.5,false);
                //go back to pick up cone
                robot.gyroTurn(90,0.5);//edit!!!
                //drive to wall...
                robot.pickUp(0.5);
                //go to low close to stack and audience then park
                robot.goToLow(0.5,true);
                //park
                robot.gyroStrafeEncoder(0.5,180,12);
                break;
            case 2:
            case 3:
            default:
                //place in low close to stack
                robot.goToLow(0.5,true);
                //go back to pickup a cone
                robot.gyroTurn(90,0.5);
                //drive to wall...
                robot.pickUp(0.5);
                //now place with respect to parking...
                if(signal == 2){
                    //place in the high closest to audience
                    robot.goToHigh(0.5,true);
                    robot.gyroStrafeEncoder(0.5,0,12);
                }
                else{
                    robot.gyroStrafeEncoder(0.5,-90,48);
                    robot.gyroTurn(180,0.5);
                    //l shape... need to test reliability of diagonal strafing
                    robot.gyroStrafeEncoder(0.5,180,12);
                    robot.gyroStrafeEncoder(0.5,90,18);
                    robot.openClaw();
                    robot.gyroStrafeEncoder(0.5, -90, 6);
                }
                break;
        }
    }
}
