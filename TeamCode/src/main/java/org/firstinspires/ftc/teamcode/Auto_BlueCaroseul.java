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

@Autonomous(name = "Blue Caroseul")
public class Auto_BlueCaroseul extends LinearOpMode {
    //robot parts
    private DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, motorOuttake, motorIntake;
    private CRServo duck;
    private Servo bucket;
    private BNO055IMU imu;

    //for robot motion
    private RobotClass robot;

    private DistanceSensor distsensefront, distsenseback;

    //for OpenCV
    OpenCvCamera cam;// webcam
    int width = 640;
    int height = 480;
    CVClass mainPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        //TODO Only proceed if you have done PIDCalibration and copied kp and kd values!!!!!!! if you are here and you haven't you be doing something wrong
        //setup robot parts
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorOuttake = hardwareMap.dcMotor.get("outtake");
        motorIntake = hardwareMap.dcMotor.get("intake");

        duck = hardwareMap.crservo.get("duck");
        bucket = hardwareMap.servo.get("bucket");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        distsenseback = hardwareMap.get(DistanceSensor.class,"distsenseback");
        distsensefront = hardwareMap.get(DistanceSensor.class,"distsensefront");

        //create robot object
        robot = new RobotClass(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, motorIntake, motorOuttake, bucket, duck, distsenseback, distsensefront, imu,this);

        //setup robot
        robot.setupRobot(true);//TODO: if motors need swapping directions, go to this method in Robot_2022FF.java and change! DO NOT CHANGE IN HERE

        //setup camera, turn it on
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
                System.exit(0);
            }
        });

        telemetry.addData("angle", robot.getAngle());
        telemetry.update();
        waitForStart();//if there is a camera error... and it crashes the program... then we need to find a way to "pause stream"

        int code = mainPipeline.getSignal();//get the code before we move
        telemetry.addData("barcode value", code);
        if (code == 0) {
            telemetry.addData("assuming", "1");
        }
        telemetry.update();

        //dist 26 cm == 10 cm here 2.6cm=1cm
        //actually in INCHES!!!! will change in a bit
        //caroseul
        robot.gyroStrafeEncoder(0.75,90,5);//to allow turning
        robot.gyroTurn(180,0.5);//direction
        robot.gyroStrafeEncoder(0.75,180,35);//issue!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // robot.goToCarousel(0.75);
        
        // robot.gyroStrafeEncoder(0.4,90,1);//2 feet+a bit more(error) to right. todo change the cm, direction
        robot.doduck(0.8);//turn on duck

        //line up with hub after duck
        robot.gyroStrafeEncoder(0.75, -90, 41);
        // robot.gyroStrafeEncoder(0.75,0,10);///dists.....
        robot.gyroTurn(87,0.5);//less?

        //drop element
        code = code<1?1:code;
        double dist = 31;//32 for 1,
        robot.runToPosDrop(0.75, dist, code);
        telemetry.addData("dropped?","yes");
        telemetry.update();
        Thread.sleep(250);
        
        //park in depot
        robot.goToDepot_Blue();
        telemetry.addData("Depot?", "yes");
        telemetry.update();
    }
}
