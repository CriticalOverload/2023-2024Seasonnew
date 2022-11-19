package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotClass2 {
    private DcMotor motorFL, motorBR, motorBL, motorFR;//our motor
    private DcMotor[] motors;//beware.... uhh
    private DcMotor viperslide;
    private Servo claw;

    //Declare an opmode and a telemetry object
    private LinearOpMode opMode;
    private Telemetry telemetry;//if we're using driver station

    //for ftcdashboard
    private TelemetryPacket packet;
    private FtcDashboard dash;
    private boolean yesDash;

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
     * @param viperslide slide motor
     * @param claw claw servo
     * @param opMode From the opMode we get telemetry
     * @param yesDash if we're using the dashboard
     * */
    public RobotClass2(DcMotor motorFL,DcMotor motorFR,DcMotor motorBL,DcMotor motorBR, DcMotor viperslide, Servo claw, LinearOpMode opMode, boolean yesDash){
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.viperslide = viperslide;
        this.claw = claw;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
        this.yesDash=yesDash;
    }

    /**
     * Base only Constructor
     * @param motorFL front left motor
     * @param motorFR front right motor
     * @param motorBL back left motor
     * @param motorBR back right motor
     * @param opMode From the opMode we get telemetry
     * @param yesDash if we're using the dashboard
     * */
    public RobotClass2(DcMotor motorFL,DcMotor motorFR,DcMotor motorBL,DcMotor motorBR, LinearOpMode opMode, boolean yesDash){
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
        this.yesDash = yesDash;
    }

    /**
     * Empty. CV testing or just for dashboard capabilities
     * */
    public RobotClass2(LinearOpMode opMode){
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.yesDash=true;
    }

    /**
     * Setup the robot for PID as well as the telemetry for FTCDashboard
     */
    public void setupRobot() throws InterruptedException{
        //reverse the needed motors here
        for(DcMotor m : motors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        resetEncoders();
        if(yesDash)
            setupDashboard();
    }

    public void setupDashboard(){
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


    //may edit this to be just for the slides todo
    public double pidTuner_noLoop(int dist, double kp, double kd, double ki, ElapsedTime timer) throws InterruptedException{
        //go distance, calculating power based on distance
        double power, error, derivative;
        double integral = 0;//might need to make this global?? or pass in?? or?????

        error = kp*(dist - motorFL.getCurrentPosition());

        derivative = kd*(error-lastError)/timer.seconds();
        lastError = error;

        integral += ki*(error*timer.seconds());

        power = error + derivative + integral;

        addData("dist",dist);
        addData("motorPos",motorFL.getCurrentPosition());
        addData("error", error);
        addData("derivative", derivative);
        addData("integral", integral);
        addData("power", power);
        update();

        return power;
    }

    public void moveSlides(int level){
        //setup this with pid stuff
        switch(level){
            default:
            case 0:
                //ground, with slight adjustment, or pick up
                break;
            case 1:
                //low
                break;
            case 2:
                //medium
                break;
            case 3:
                //high
                break;
        }
    }

    public void openClaw(){
        claw.setPosition(1);
    }

    public void closeClaw(){
        claw.setPosition(0);
    }

    public void setLastError(double e){//?????????
        lastError= e;
    }

    public void addData(String header, Object data){
        packet.put(header, data);
    }

    public void update(){
        dash.sendTelemetryPacket(packet);
    }
}