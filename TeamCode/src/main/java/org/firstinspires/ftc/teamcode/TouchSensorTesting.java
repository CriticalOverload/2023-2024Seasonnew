package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

//@Autonomous(name="Touch Sensor Testing")
public class TouchSensorTesting extends LinearOpMode {
    private TouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException{
        touch = hardwareMap.get(TouchSensor.class, "touchSensor");
        waitForStart();
        telemetry.addData("Touch value",touch.getValue());
        telemetry.addData("Touch pressed",touch.isPressed());
        telemetry.update();
    }
}
