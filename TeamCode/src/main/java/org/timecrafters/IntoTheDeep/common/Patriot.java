package org.timecrafters.IntoTheDeep.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorSparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;
@Config
public class Patriot implements Robot {

    private HardwareMap hardwareMap;
    public TimeCraftersConfiguration configuration;
    private String string;
    private CyberarmEngine engine;
    public DcMotorEx fl, fr, bl, br, intakeExtendo, depositRightExtendo, depositLeftExtendo;
    public CRServo leftDiff, rightDiff;
    public Servo depoClaw, depoLeft, depoRight, intakeClaw;
    public SparkFunOTOS otos;
    public final int RIGHT_DIFFY = 1;
    public final int LEFT_DIFFY = 0;
    public double leftDiffyPower;
    public double rightDiffyPower;

    // Declare the OctoQuad object and members to store encoder positions and velocities
    public OctoQuad octoquad;
    public int posLeftDiffy;
    public int posRightDiffy;
    public IMU imu;

    // ---------------------------------------------------------------------------------------------------- Mechanical positions and limits
    public double depoPivotPos = 0;
    public double IntakeCLOSE = 1;
    public double DepoCLOSE = 1;
    public double IntakeOPEN = 0.4;
    public double DepoOPEN = 0.7;
    public int intakeExposed = 5100;

    // Odometry driving Variables
    // -----------------------------------------------------------------------------------------------------------------------------------
    private double maxVelocity = 2320 * 0.8;
    public double posX;
    public double posY;
    public double posH;
    public double xTarget = 0;
    public double yTarget = 0;
    public double hTarget = 0;
    public double xMaxPower = 1;
    public double yMaxPower = 1;
    public double hMaxPower = 1;

    // heading math variables for pid with imu
    public static double Hp = 0.7, Hi = 0, Hd = 0;
    public static double Xp = 0.04, Xi = 0, Xd = 0;
    public static double Yp = 0.04, Yi = 0, Yd = 0;
    public double pidX;
    public double pidY;
    public double pidH;

    public double rawPidX;
    public double rawPidY;
    public double rawPidH;

    public double headingIntegralSum = 0;
    public double XIntegralSum = 0;
    public double YIntegralSum = 0;
    private double headingLastError = 0;
    private double XLastError = 0;
    private double YLastError = 0;
    ElapsedTime headingTimer = new ElapsedTime();
    ElapsedTime XTimer = new ElapsedTime();
    ElapsedTime YTimer = new ElapsedTime();

    // -----------------------------------------
    // Horizontal Extendo Variables
    public static double extendoP = 0.005, extendoI = 0, extendoD = 0;

    public double ExtendoIntegralSum = 0;
    private double ExtendoLastError = 0;
    ElapsedTime ExtendoTimer = new ElapsedTime();
    public int extendoTarget;
    public double rawPidExtendo;
    public double pidExtendo;



    public Patriot(String string) {
        this.engine = engine;
        setup();
        this.string = string;
    }

    public void setup() {
        System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;

        // Get a reference to the sensor
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        configureOtos();

        // IMU
        imu = engine.hardwareMap.get(IMU.class, "imu_ex");

        // OCTOQUAD
        octoquad = engine.hardwareMap.get(OctoQuad.class, "octoquad");
        octoquad.setSingleEncoderDirection(LEFT_DIFFY, OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(RIGHT_DIFFY, OctoQuad.EncoderDirection.REVERSE);

        //motors configuration
        intakeExtendo = (DcMotorEx) engine.hardwareMap.dcMotor.get("extension");
        depositLeftExtendo = (DcMotorEx) engine.hardwareMap.dcMotor.get("leftLift");
        depositRightExtendo = (DcMotorEx) engine.hardwareMap.dcMotor.get("rightLift");

        depositLeftExtendo.setDirection(DcMotorSimple.Direction.REVERSE);


        fl = (DcMotorEx) engine.hardwareMap.dcMotor.get("fl");
        fr = (DcMotorEx) engine.hardwareMap.dcMotor.get("fr");
        br = (DcMotorEx) engine.hardwareMap.dcMotor.get("br");
        bl = (DcMotorEx) engine.hardwareMap.dcMotor.get("bl");

        fl.setDirection(DcMotorEx.Direction.FORWARD);
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fr.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        br.setDirection(DcMotorEx.Direction.REVERSE);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));


        imu.initialize(parameters);

        // SERVO'S
        intakeClaw = engine.hardwareMap.servo.get("theClaw");
        depoClaw = engine.hardwareMap.servo.get("depo claw");
        depoLeft = engine.hardwareMap.servo.get("depo left");
        depoRight = engine.hardwareMap.servo.get("depo right");
        depoRight.setDirection(Servo.Direction.FORWARD);
        depoLeft.setDirection(Servo.Direction.REVERSE);

        // CR SERVO'S
        leftDiff = engine.hardwareMap.crservo.get("leftDiff");
        rightDiff = engine.hardwareMap.crservo.get("rightDiff");
        leftDiff.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDiff.setDirection(DcMotorSimple.Direction.REVERSE);

        configuration = new TimeCraftersConfiguration("Patriot");


    }

    public void OdoLocalizer() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        posX = pos.x;
        posY = pos.y;
        posH = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public double HeadingPIDControl(double reference, double current) {
        double error = angleWrap(current - reference);
        headingIntegralSum += error * headingTimer.seconds();
        double derivative = (error - headingLastError) / headingTimer.seconds();

        headingTimer.reset();

        double output = (error * Hp) + (derivative * Hd) + (headingIntegralSum * Hi);
        return output;
    }

    public double XPIDControl(double reference, double current) {
        double error = (reference - current);
        XIntegralSum += error * XTimer.seconds();
        double derivative = (error - XLastError) / XTimer.seconds();

        XTimer.reset();

        double output = (error * Xp) + (derivative * Xd) + (XIntegralSum * Xi);
        return output;
    }

    public double YPIDControl(double reference, double current) {
        double error = (reference - current);
        YIntegralSum += error * YTimer.seconds();
        double derivative = (error - YLastError) / YTimer.seconds();

        YTimer.reset();

        double output = (error * Yp) + (derivative * Yd) + (YIntegralSum * Yi);
        return output;
    }

    public void YDrivePowerModifier() {

        rawPidY = YPIDControl(yTarget, posY);

        if (Math.abs(rawPidY) > yMaxPower) {
            if (rawPidY < 0) {
                pidY = -yMaxPower;
            } else {
                pidY = yMaxPower;
            }
        } else {
            pidY = rawPidY;
        }
    }

    public void XDrivePowerModifier() {

        rawPidX = XPIDControl(xTarget, posX);

        if (Math.abs(rawPidX) > xMaxPower) {
            if (rawPidX < 0) {
                pidX = -xMaxPower;
            } else {
                pidX = xMaxPower;
            }
        } else {
            pidX = rawPidX;
        }
    }

    public void HDrivePowerModifier() {

        rawPidH = HeadingPIDControl(Math.toRadians(hTarget), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        if (Math.abs(rawPidH) > hMaxPower) {
            if (rawPidH < 0) {
                pidH = -hMaxPower;
            } else {
                pidH = hMaxPower;
            }
        } else {
            pidH = rawPidH;
        }
    }

    public void DriveToCoordinates() {
        // determine the powers needed for each direction
        // this uses PID to adjust needed Power for robot to move to target

        double y = pidY;
        double x = -pidX; // Counteract imperfect strafing
        double rx = -pidH;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // angle math to make things field oriented
        double heading = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        double frontLeftVelo = maxVelocity * ((rotY + rotX + rx) / denominator);
        double backLeftVelo = maxVelocity * ((rotY - rotX + rx) / denominator);
        double frontRightVelo = maxVelocity * ((rotY - rotX - rx) / denominator);
        double backRightVelo = maxVelocity * ((rotY + rotX - rx) / denominator);

        // setting each power determined previously from the math above
        // as well as multiplying it by a drive power that can be changed.

        // apply my powers
        bl.setVelocity(backLeftVelo);
        br.setVelocity(backRightVelo);
        fl.setVelocity(frontLeftVelo);
        fr.setVelocity(frontRightVelo);
    }

    private void configureOtos() {

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        otos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
        
    }

    public double ExtendoPID(double reference, double current) {
        double error = (reference - current);
        ExtendoIntegralSum += error * ExtendoTimer.seconds();
        double derivative = (error - ExtendoLastError) / ExtendoTimer.seconds();

        ExtendoTimer.reset();

        double output = (error * extendoP) + (derivative * extendoD) + (ExtendoIntegralSum * extendoI);
        return output;
    }
    public void HorizontalExtendoControl(){

        rawPidExtendo = ExtendoPID(extendoTarget, intakeExtendo.getCurrentPosition());

        if (Math.abs(rawPidExtendo) > 1) {
            if (rawPidExtendo < 0) {
                pidExtendo = -1;
            } else {
                pidExtendo = 1;
            }
        } else {
            pidExtendo = rawPidExtendo;
        }

        if ((intakeExtendo.getCurrentPosition() < extendoTarget + 50) && (intakeExtendo.getCurrentPosition() > extendoTarget - 50)){
            intakeExtendo.setVelocity(0);
        } else {
            intakeExtendo.setVelocity((1760 * 0.95) * pidExtendo);
        }
    }

    public void readOctoQuad(){
        int[] positions = octoquad.readAllPositions();
        posLeftDiffy  = positions[LEFT_DIFFY];
        posRightDiffy = positions[RIGHT_DIFFY];
    }
}
