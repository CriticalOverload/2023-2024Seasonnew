package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "Testing to make sure it works")
@Config
public class TestingAutoThing extends LinearOpMode {

    @Override//not strictly needed but keeping here
    public void runOpMode(){
        RobotClass2 dashboard = new RobotClass2(this);
        dashboard.setupDashboard();
        int i = 0;
        waitForStart();
        while(opModeIsActive()){
            dashboard.addData("time",i++);
            dashboard.update();
        }
    }
}
