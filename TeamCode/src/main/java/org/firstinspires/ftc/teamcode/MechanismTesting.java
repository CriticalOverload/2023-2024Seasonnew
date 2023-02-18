package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Mechanism Test")
public class MechanismTesting extends LinearOpMode {
    private DcMotor slide;
    private Servo claw;

    @Override
    public void runOpMode() throws InterruptedException{
        slide = hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");

        waitForStart();
        while(opModeIsActive()){
            slide.setPower(gamepad1.right_stick_y);
            if(gamepad1.a){
                claw.setPosition(1.0);
            }
            if(gamepad1.b) {
                claw.setPosition(0.0);
            }
        }

    }
}
