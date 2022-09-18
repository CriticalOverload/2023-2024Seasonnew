package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotClass2 {
    private DcMotor motorFL, motorBR, motorBL, motorFR;//our motor
    private DcMotor[] motors;

    //Declare an opmode and a telemetry object
    private LinearOpMode opMode;
    private Telemetry telemetry;//if we're using driver station

    //for ftcdashboard
    private TelemetryPacket packet;
    private FtcDashboard dash;

    //for PID
    private double lastError=0;
    private double ckp, cki, ckd;//there is a c in front to remind me to CHANGE!!! TODO!!!!!!!!!!!

    //setup
    /**
     * Full Constructor
     * @param motorFL front left motor
     * @param motorFR front right motor
     * @param motorBL back left motor
     * @param motorBR back right motor
     * @param opMode From the opMode we get telemetry
     * */
    public RobotClass2(DcMotor motorFL,DcMotor motorFR,DcMotor motorBL,DcMotor motorBR, LinearOpMode opMode){
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
    }

    /**
     * Setup the robot for PID as well as the telemetry for FTCDashboard
     */
    public void setupRobot() throws InterruptedException{
        //reverse the needed motors?
        for(DcMotor m : motors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        resetEncoders();

        //for dashboard vvv
        packet = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        packet.put("setup","done");
        dash.sendTelemetryPacket(packet);
    }

    /**
     * Reset motor encoders
     */
    public void resetEncoders(){
        for(DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lastError = 0;
    }


    //robot motion
    /**
     * Set motor powers to 0 (complete stop)
     */
    public void completeStop(){
        for(DcMotor m : motors) {
            m.setPower(0);
        }
    }

    /**
     * Straight Mecanum drive (no turns, allows strafes)
     * @param leftPower, for FL and BR
     * @param rightPower, for FR and BL
     * */
    public void tankDrive(double leftPower, double rightPower){
        motorFL.setPower(leftPower);
        motorFR.setPower(rightPower);
        motorBL.setPower(rightPower);
        motorBR.setPower(leftPower);
    }

    public double pidTuner_noLoop(int dist, double kp, double kd, double ki, ElapsedTime timer) throws InterruptedException{
        //go distance, calculating power based on distance
        double power, error, derivative;
        double integral = 0;//might need to make this global?? or pass in?? or?????

        error = kp*(dist - motorFL.getCurrentPosition());

        derivative = kd*(error-lastError)/timer.seconds();
        lastError = error;

        integral += ki*(error*timer.seconds());

        power = error + derivative + integral;

        addTelem("dist",dist);
        addTelem("motorPos",motorFL.getCurrentPosition());
        addTelem("error", error);
        addTelem("derivative", derivative);
        addTelem("integral", integral);
        addTelem("power", power);
        sendTelem();

        return power;
    }


    public void setLastError(double e){//?????????
        lastError= e;
    }

    public void addTelem(String header, Object data){
        packet.put(header, data);
    }

    public void sendTelem(){
        dash.sendTelemetryPacket(packet);
    }
}