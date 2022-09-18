package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "distance test")
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor distsensefront, distsenseback;

    @Override
    public void runOpMode() throws InterruptedException{

        distsensefront = hardwareMap.get(DistanceSensor.class,"distsensefront");
        distsenseback = hardwareMap.get(DistanceSensor.class,"distsenseback");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("deviceName","front");
            telemetry.addData("distance reading:", distsensefront.getDistance(DistanceUnit.CM));
            telemetry.addData("deviceName","back");
            telemetry.addData("distance reading:", distsenseback.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}