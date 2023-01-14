package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotClass2 {
    private DcMotor motorFL, motorBR, motorBL, motorFR, motorLS;//our motor
    private DcMotor[] motors;//beware.... uhh
    private DcMotor viperslide;
    private Servo claw;

    private DcMotor leftEncoder, rightEncoder, backEncoder;
    boolean deadWheels = false;

    //imu stuff
    private BNO055IMU imu;
    private Orientation angles, lastAngles, startAngles;
    private double globalAngle;

    //Declare an opmode and a telemetry object
    private LinearOpMode opMode;
    private Telemetry telemetry;//if we're using driver station

    //for ftcdashboard
    private TelemetryPacket packet;
    private FtcDashboard dash;
    private boolean yesDash;

    //for PID
    private double lastError = 0;
    private double ckp, cki, ckd;//there is a c in front to remind me to CHANGE!!! TODO!!!!!!!!!!!
    private double integral = 0;

    private double robotIntegral = 0;
    private double robotLastError = 0;

    //drive distance calculation
    private final double DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * 3.77953;
    //
    // private final double MOTOR_RPM = 435;
    // private final double MOTOR_SPR = IDK WHAT THIS IS 60/MOTOR_RPM;//CHANGE
    // private final double SECONDS_PER_CM = IDK WHAT THIS IS EITEHR MOTOR_SPR/DRIVE_WHEEL_CIRCUMFERENCE;
    private final double TICKS_PER_REV = (1 + (double) 46 / 17) * (1 + (double) 46 / 17) * 28;
    private final double TICKS_PER_IN = TICKS_PER_REV / DRIVE_WHEEL_CIRCUMFERENCE;

    // private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.Pi * diameter!!!!!!!!;
    // private final double ENCODER_TICKS_PER_REV = INSERTNUMBERHERE;
    private final double ENCODER_TICKS_PER_IN = 307.699557;

    //setup

    /**
     * Full Constructor
     *
     * @param motorFL      front left motor
     * @param motorFR      front right motor
     * @param motorBL      back left motor
     * @param motorBR      back right motor
     * @param leftEncoder  Left encoder
     * @param rightEncoder right encoder
     * @param backEncoder  back encoder
     * @param viperslide   slide motor
     * @param claw         claw servo
     * @param imu          imu
     * @param opMode       From the opMode we get telemetry
     * @param yesDash      if we're using the dashboard
     */
    public RobotClass2(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, DcMotor leftEncoder, DcMotor rightEncoder, DcMotor backEncoder, DcMotor viperslide, Servo claw, BNO055IMU imu, LinearOpMode opMode, boolean yesDash) {
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.backEncoder = backEncoder;

        this.viperslide = viperslide;
        this.claw = claw;
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
        this.yesDash = yesDash;
        deadWheels = true;
    }

    /**
     * Full Constructor without encoders
     *
     * @param motorFL    front left motor
     * @param motorFR    front right motor
     * @param motorBL    back left motor
     * @param motorBR    back right motor
     * @param viperslide slide motor
     * @param claw       claw servo
     * @param imu        imu
     * @param opMode     From the opMode we get telemetry
     * @param yesDash    if we're using the dashboard
     */
    public RobotClass2(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, DcMotor viperslide, Servo claw, BNO055IMU imu, LinearOpMode opMode, boolean yesDash) {
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.viperslide = viperslide;
        this.claw = claw;
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
        this.yesDash = yesDash;
    }

    /**
     * Full Constructor without encoders
     *
     * @param motorFL    front left motor
     * @param motorFR    front right motor
     * @param motorBL    back left motor
     * @param motorBR    back right motor
     * @param viperslide slide motor
     * @param claw       claw servo
     * @param opMode     From the opMode we get telemetry
     * @param yesDash    if we're using the dashboard
     */
    public RobotClass2(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, DcMotor viperslide, Servo claw, LinearOpMode opMode, boolean yesDash) {
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.viperslide = viperslide;
        this.claw = claw;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
        this.yesDash = yesDash;
    }

    /**
     * Base only Constructor
     *
     * @param motorFL front left motor
     * @param motorFR front right motor
     * @param motorBL back left motor
     * @param motorBR back right motor
     * @param imu     imu
     * @param opMode  From the opMode we get telemetry
     * @param yesDash if we're using the dashboard
     */
    public RobotClass2(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, BNO055IMU imu, LinearOpMode opMode, boolean yesDash) {
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
        this.yesDash = yesDash;
    }

    /**
     * Base only + encoders Constructor
     *
     * @param motorFL      front left motor
     * @param motorFR      front right motor
     * @param motorBL      back left motor
     * @param motorBR      back right motor
     * @param leftEncoder  Left encoder
     * @param rightEncoder right encoder
     * @param backEncoder  back encoder
     * @param imu          imu
     * @param opMode       From the opMode we get telemetry
     * @param yesDash      if we're using the dashboard
     */
    public RobotClass2(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, DcMotor leftEncoder, DcMotor rightEncoder, DcMotor backEncoder, BNO055IMU imu, LinearOpMode opMode, boolean yesDash) {
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.backEncoder = backEncoder;
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
        this.yesDash = yesDash;
        deadWheels = true;
    }

    /**
     * Base only no IMU Constructor
     *
     * @param motorFL front left motor
     * @param motorFR front right motor
     * @param motorBL back left motor
     * @param motorBR back right motor
     * @param opMode  From the opMode we get telemetry
     * @param yesDash if we're using the dashboard
     */
    public RobotClass2(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, LinearOpMode opMode, boolean yesDash) {
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
     */
    public RobotClass2(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.yesDash = true;
    }

    /**
     * Empty with Slides. For PID callibration only!
     */
    public RobotClass2(DcMotor viperslide, LinearOpMode opMode, boolean yesDash) {
        this.viperslide = viperslide;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.yesDash = yesDash;
    }

    /**
     * Setup the robot and the telemetry for FTCDashboard
     */
    public void setupRobot() throws InterruptedException {
        reverseMotors();
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setIMUParameters();
        resetEncoders();
        resetSlides();
        resetAngle();

        if (yesDash)
            setupDashboard();

        while (!imu.isGyroCalibrated()) {
            if (yesDash) {
                addData("IMU", "calibrating...");
                update();
                sleep(50);
            } else {
                telemetry.addData("IMU", "calibrating...");
                telemetry.update();
                sleep(50);
            }
        }

        telemetry.addData("IMU", "ready");
        telemetry.update();
    }

    /**
     * Setup the robot and the telemetry for FTCDashboard
     */
    public void setupRobot_base() throws InterruptedException {
        reverseMotors();
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setIMUParameters();
        resetEncoders();
        resetAngle();

        if (yesDash)
            setupDashboard();

        while (!imu.isGyroCalibrated()) {
            if (yesDash) {
                addData("IMU", "calibrating...");
                update();
                sleep(50);
            } else {
                telemetry.addData("IMU", "calibrating...");
                telemetry.update();
                sleep(50);
            }
        }

        telemetry.addData("IMU", "ready");
        telemetry.update();
    }

    /**
     * Setup the robot and the telemetry for FTCDashboard
     */
    public void setupRobot_base_noimu() throws InterruptedException {
        reverseMotors();
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();

        if (yesDash)
            setupDashboard();

        telemetry.addData("IMU", "ready");
        telemetry.update();
    }


    /**
     * Setup the robot and the telemetry for FTCDashboard
     */
    public void setupRobot_base_slide_noimu() throws InterruptedException {
        reverseMotors();
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        resetSlides();

        if (yesDash)
            setupDashboard();

        telemetry.addData("IMU", "ready");
        telemetry.update();
    }

    public void reverseMotors() throws InterruptedException {
        //reverse needed motors here!!
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Setup to use FTCDashboard
     */
    public void setupDashboard() {
        packet = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        packet.put("setup", "done");
        dash.sendTelemetryPacket(packet);
    }

    /**
     * Reset motor encoders
     */
    public void resetEncoders() {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (deadWheels) {
            leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        lastError = 0;
    }

    /**
     * Reset the slides to run to position
     */
    public void resetSlides() {
        viperslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperslide.setTargetPosition(0);
        viperslide.setPower(0);
        viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Reset angle measurement
     */
    public void resetAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngles = lastAngles;
        globalAngle = 0;
    }


    /**
     * Get IMU parameters
     *
     * @return IMU parameters
     */
    private BNO055IMU.Parameters getIMUParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.IMU;

        return parameters;
    }

    /**
     * Setup and initialize IMU parameters
     */
    private void setIMUParameters() {
        BNO055IMU.Parameters parameters = getIMUParameters();
        imu.initialize(parameters);
    }


    //imu stuff

    /**
     * Get change in angle since last reset
     *
     * @return angle in degrees (+ CCW, - CW)
     */
    public double getAngle() {
        //Get a new angle measurement
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Get the difference between current angle measurement and last angle measurement
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //Process the angle to keep it within (-180,180)
        //(Once angle passes +180, it will rollback to -179, and vice versa)
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //Add the change in angle since last measurement (deltaAngle)
        //to the change in angle since last reset (globalAngle)
        globalAngle += deltaAngle;
        //Set last angle measurement to current angle measurement
        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Create telemetry with angle after last reset, current angle, and change since last reset
     */
    private void composeAngleTelemetry() {
        telemetry.addData("Start Angle", startAngles.firstAngle);
        telemetry.addData("Current Angle", angles.firstAngle);
        telemetry.addData("Global Angle", globalAngle);
    }


    //robot motion

    /**
     * Set motor powers to 0 (complete stop)
     */
    public void completeStop() {
        for (DcMotor m : motors) {
            m.setPower(0);
        }
    }

    /**
     * Straight Mecanum drive (no turns, allows strafes)
     *
     * @param leftPower,  for FL and BR
     * @param rightPower, for FR and BL
     */
    public void tankDrive(double leftPower, double rightPower) {
        motorFL.setPower(leftPower);
        motorFR.setPower(rightPower);
        motorBL.setPower(rightPower);
        motorBR.setPower(leftPower);
    }

    /**
     * Straight mecanum drive with correction
     *
     * @param leftPower  for FL and BR
     * @param rightPower for FR and BL
     * @param correction
     */
    public void correctedTankStrafe(double leftPower, double rightPower, double correction) {
        motorFL.setPower(leftPower - correction);
        motorFR.setPower(rightPower + correction);
        motorBL.setPower(rightPower - correction);
        motorBR.setPower(leftPower + correction);
    }

    /**
     * Generic turn function, works for CW and CCW
     *
     * @param power power for motors, + is CCW, - is CW
     */
    public void turn(double power) {
        motorFL.setPower(-power);
        motorBL.setPower(-power);
        motorFR.setPower(power);
        motorBR.setPower(power);
    }

    /**
     * Make precise turn using gyro
     * no PID
     * + is ccw, - is cw
     *
     * @param degrees
     * @param power
     */
    public void gyroTurn(int degrees, double power) throws InterruptedException {
        //restart angle tracking
        resetAngle();
        if (degrees == 0)
            return;
        if (degrees < 0) {
            turn(power * -1);
        } else {
            turn(power);
        }

        //Rotate until current angle is equal to the target angle
        //getAngle()-degrees
        /*while(opMode.opModeIsActive() && Math.abs(getAngle()-degrees) > 10){
            composeAngleTelemetry();
            telemetry.addData("Target angle", degrees);
            telemetry.update();
        }*///doesn't work the way we want it to... may edit later but not urgent todo
        if (degrees < 0) {
            while (opMode.opModeIsActive() && getAngle() > degrees + 10) {
                composeAngleTelemetry();
                //display the target angle
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        } else {
            while (opMode.opModeIsActive() && getAngle() < degrees - 10) {
                composeAngleTelemetry();
                //display the target angle
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        }

        completeStop();
        //Wait .5 seconds to ensure robot is stopped before continuing
        sleep(250);
        resetAngle();
    }


    /**
     * Make precise turn using gyro
     * no PID
     * + is ccw, - is cw
     *
     * @param degrees
     * @param power
     */
    public void encoderTurn(int degrees, double power) throws InterruptedException {
        //restart angle tracking
        resetAngle();
        if (degrees == 0)
            return;
        if (degrees < 0) {
            turn(power * -1);
        } else {
            turn(power);
        }

        //Rotate until current angle is equal to the target angle
        //getAngle()-degrees
        /*while(opMode.opModeIsActive() && Math.abs(getAngle()-degrees) > 10){
            composeAngleTelemetry();
            telemetry.addData("Target angle", degrees);
            telemetry.update();
        }*///doesn't work the way we want it to... may edit later but not urgent todo
        if (degrees < 0) {
            while (opMode.opModeIsActive() && getAngle() > degrees + 10) {
                composeAngleTelemetry();
                //display the target angle
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        } else {
            while (opMode.opModeIsActive() && getAngle() < degrees - 10) {
                composeAngleTelemetry();
                //display the target angle
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        }

        completeStop();
        //Wait .5 seconds to ensure robot is stopped before continuing
        sleep(250);
        resetAngle();
    }

    //add pid gyro turn?????....

    /**
     * Check if robot is moving in straight line. If not, then get a power correction
     *
     * @return correction, +=CW, -=CCW
     */
    public double getCorrection() {
        //Get the current angle of the robot
        double angle = getAngle();

        //Use the angle to calculate the correction
        if (angle == 0) {
            //If angle = 0, robot is moving straight; no correction needed
            return 0;
        } else {
            //If angle != 0, robot is not moving straight
            //Correction is negative angle (to move the robot in the opposite direction)
            //multiplied by gain; the gain is the sensitivity to angle
            //We have determined that .1 is a good gain; higher gains result in overcorrection
            //Lower gains are ineffective
            return -angle * 0.02;//gain;<<<< make var?? todo do we even need gain?
        }
    }

    /**
     * Strafe in any direction using encoders.
     * use this
     *
     * @param power
     * @param angle Direction to strafe (90 = forward, 0 = right, -90 = backwards, 180 = left)
     * @param in    inches
     * @throws InterruptedException if robot is stopped
     */
    public void gyroStrafeEncoder(double power, double angle, double in) throws InterruptedException {
        double ticks = in * TICKS_PER_IN;

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI / 180 - Math.PI / 4;

        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;
        telemetry.addData("left", leftPower);
        telemetry.addData("right", rightPower);
        telemetry.update();
        resetEncoders();
        resetAngle();
//        setNewGain(0.02);

        while (Math.abs(motorFL.getCurrentPosition()) < ticks && opMode.opModeIsActive()) {
            double correction = getCorrection();
            composeAngleTelemetry();
            telemetry.addData("correction", correction);
            telemetry.update();
            correctedTankStrafe(leftPower, rightPower, correction);
        }
        completeStop();
        sleep(250);
        resetAngle();
        resetEncoders();
    }

    /**
     * Strafe in any direction using encoders.
     * use this
     *
     * @param power
     * @param angle Direction to strafe (90 = forward, 0 = right, -90 = backwards, 180 = left)
     * @param in    inches
     * @throws InterruptedException if robot is stopped
     */
    public void gyroStrafeEncoder_deadWheels(double power, double angle, double in) throws InterruptedException {
        double ticks = in * ENCODER_TICKS_PER_IN;//Todo CHANGE FOR DEADWHEELS!!!!!!!!


        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI / 180 - Math.PI / 4;

        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;

        resetEncoders();
        resetAngle();
        //setNewGain(0.02);
        DcMotor encoder;
        //gyroStrafeEncoder_deadWheels(1, -90, 3); newDirection = -3p/4
        //gyroStrafeEncoder_deadWheels(1, 90, 3); newDirection = p/4
        //gyroStrafeEncoder_deadWheels(1, 0, 3); newDirection = -p/4
        //gyroStrafeEncoder_deadWheels(1, 180, 3); newDirection = 3p/4

        if (newDirection == Math.PI / 4 || newDirection == -3 * Math.PI / 4) {
            encoder = leftEncoder;
        } else {
            encoder = backEncoder;
        }


        while (Math.abs(encoder.getCurrentPosition()) < ticks && opMode.opModeIsActive()) {
            double correction = getCorrection();
            correctedTankStrafe(leftPower, rightPower, correction);
            //todo adjust to have PID tuning stuff
        }
        completeStop();
        sleep(250);
        resetAngle();
        resetEncoders();
    }


    /**
     * Strafe in any direction using encoders.
     * use this
     *
     * @param power
     * @param angle Direction to strafe (90 = forward, 0 = right, -90 = backwards, 180 = left)
     * @param in    inches
     * @throws InterruptedException if robot is stopped
     */
    public void gyroStrafeEncoder_noimu(double power, double angle, double in) throws InterruptedException {
        double ticks = in * TICKS_PER_IN;

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI / 180 - Math.PI / 4;

        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;

        resetEncoders();
        // resetAngle();
        //setNewGain(0.02);

        while (Math.abs(motorFL.getCurrentPosition()) < ticks && opMode.opModeIsActive()) {
            // double correction = getCorrection();
            tankDrive(leftPower, rightPower);
        }
        completeStop();
        sleep(250);
        // resetAngle();
        resetEncoders();
    }

    //auto movements and actions
    public void goToHigh(double power, boolean blue) throws InterruptedException {
        // moveSlides(3,power);
        gyroStrafeEncoder(power, -90, 34);
        if (blue) {
            gyroTurn(90, power);
        } else {
            gyroTurn(-90, power);
        }
        gyroStrafeEncoder(power, 90, 4);
        openClaw();
    }

    public void pickUp(double power) {
        // moveSlides(0,power);
        closeClaw();
    }

    public void goToLow(double power, boolean blue) throws InterruptedException {
        // moveSlides(1,power);
        gyroStrafeEncoder(power, -90, 22);
        if (blue) {
            gyroTurn(-90, power);
        } else {
            gyroTurn(90, power);
        }
        gyroStrafeEncoder(power, 0, 6);
        gyroStrafeEncoder(power, 90, 2);
        openClaw();
        gyroStrafeEncoder(power, -90, 2);
        closeClaw();
        gyroStrafeEncoder(power, 180, 6);
        if (blue) {
            gyroTurn(90, power);
        } else {
            gyroTurn(-90, power);
        }
    }

    /**
     * used by terminal autos to get from starting position to terminal
     */
    public void dropInTerminal(double power, boolean blue) throws InterruptedException {
        int mod = 1;
        if (blue)
            mod = -1;
        gyroStrafeEncoder(power * mod, 0, 28);//may need more, basically go to the wall
        openClaw();
        sleep(500);



        //lift up slide TODO
    }

    //attachments

    /**
     * Move slide to position
     *
     * @param level ground hub thing/(cone on) ground(slight adjustment), low, medium, or high junction
     */
    public void moveSlides(int level, double power) {
        double circumference = 4.409;//circumference of pulley for hub
//        double groundRN = ; // rotations needed to reach point from 0
//        double smallRN =;
        double medRN = 26 / circumference;
        double highEP = 37 / circumference;// (inches from ground / circumference)* ticks

        //setup this with pid stuff
        int target;
        switch (level) {
            default:
            case 0:
                //pick up
                target = -389;//2/circumference;//arbitrary todo change!!!
                break;
            case 1:
                //low
                target = -1685;//16/circumference;
                break;
            case 2:
                //medium
                target = 2891;
                break;
            case 3:
                //high
                target = -4065;
                break;
            case 4:
                //ground
                target = -36;
                break;
            case 5:
                //4cones
                target = -803;
                break;
            case 6:
                //4cones pickup height
                target = -750;
                break;

        }
        viperslide.setPower(power);
        viperslide.setTargetPosition(target);
        while (Math.abs(viperslide.getCurrentPosition() - target) > 5 && opMode.opModeIsActive()) ;
        viperslide.setPower(0);
    }

    public void openClaw() {
        claw.setPosition(0.5);
    }

    public void closeClaw() {
        claw.setPosition(-1);
    }

    public void setLastError(double e) {//?????????
        lastError = e;
    }

    //dashboard

    public void addData(String header, Object data) {
        packet.put(header, data);
    }

    public void update() {
        dash.sendTelemetryPacket(packet);
    }

    //pid section

    /**
     * Reset slides PID. MUST DO BEFORE EACH CALL!!!!!
     */
    public void resetPID() {
        integral = 0;
        lastError = 0;
    }

    /**
     * Tuning PID for the slides
     */
    public double pidTuner_noLoop(int dist, double kp, double kd, double ki, ElapsedTime timer) {
        //go distance, calculating power based on distance
        double power, error, derivative;

        error = dist - viperslide.getCurrentPosition();

        derivative = (error - lastError) / timer.seconds();
        lastError = error;

        integral += error * timer.seconds();

        power = error * kp + derivative * kd + integral * ki;


//        addData("dist",dist);
//        addData("motorPos",viperslide.getCurrentPosition());
//        addData("error", error);
//        addData("derivative", derivative);
//        addData("integral", integral);
//        addData("power", power);
//        update();
        return power;
    }

    /**
     * Reset slides PID. MUST DO BEFORE EACH CALL!!!!!
     */
    public void resetRobotPID() {
        robotIntegral = 0;
        robotLastError = 0;
    }

    /**
     * Tuning PID for the robot
     */
    public double robotPidTuner_noLoop(int dist, double kp, double kd, double ki, DcMotor encoder, ElapsedTime timer) {
        //go distance, calculating power based on distance
        double power, error, derivative;

        error = dist - encoder.getCurrentPosition();//??

        derivative = (error - robotLastError) / timer.seconds();
        robotLastError = error;

        robotIntegral += error * timer.seconds();

        power = error * kp + derivative * kd + robotIntegral * ki;

//        addData("dist",dist);
//        addData("motorPos",viperslide.getCurrentPosition());
//        addData("error", error);
//        addData("derivative", derivative);
//        addData("integral", integral);
//        addData("power", power);
//        update();
        return power;
    }
}
