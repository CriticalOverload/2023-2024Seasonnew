package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "PID Calib")
@Config
public class PIDCalibration extends LinearOpMode {
    private DcMotor slides;

    private RobotClass2 robot;
    public static double kp = 0, kd = 0, ki = 0;
    public static int dist = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = hardwareMap.dcMotor.get("ls");

        robot = new RobotClass2(slides,this, true);
        robot.setupRobot();

        kp = 1;
        kd = 0;
        ki = 0;
        //todo: copy numbers over torobotclass 2 when done!

        waitForStart();
        boolean auto = false;//note: code starts as driver controlled
        boolean goRun = false;
        int switchInt = -1;

        while(opModeIsActive()){
            if(auto){
                if(goRun) {
                    ElapsedTime timer = new ElapsedTime();
                    while(Math.abs(slides.getCurrentPosition() - dist) > 5 && !gamepad1.right_bumper) {
                        double power = robot.pidTuner_noLoop(switchInt * dist, kp, kd, ki, timer);
                        slides.setPower(power);
                    }
                    goRun = false;
                    Thread.sleep(5000);
                }
            }
            else {
                slides.setPower(gamepad1.right_stick_y);
            }

            if(gamepad1.right_bumper) {//left bumper to switch modes
                auto = !auto;
                Thread.sleep(250);
            }
            if(gamepad1.left_bumper) {//right bumper to change directions. In theory, after reaching destination it should stop... todo if it shakes/doesn't stop let me know!
                switchInt = -1 * switchInt;
                goRun = true;
                Thread.sleep(250);
            }
            if(gamepad1.a) {//increase kp
                kp+=0.1;
                Thread.sleep(250);
            }
            if(gamepad1.b) {//decrease kp
                kp-=0.1;
                Thread.sleep(250);
            }
            if(gamepad1.x) {//increase kd
                kd+=0.1;
                Thread.sleep(250);
            }
            if(gamepad1.y) {//decrease kd
                kd-=0.1;
                Thread.sleep(250);
            }
            if(gamepad1.dpad_up){//increase ki
                ki += 0.1;
                Thread.sleep(250);
            }
            if(gamepad1.dpad_down){//decrease ki
                ki -= 0.1;
                Thread.sleep(250);
            }

            telemetry.addData("kp", kp);
            telemetry.addData("ki", ki);
            telemetry.addData("kd", kd);
            telemetry.addData("run?", goRun);
            telemetry.addData("direction?", switchInt);
            telemetry.addData("in auto mode?", auto);
            telemetry.update();
        }
    }
}
