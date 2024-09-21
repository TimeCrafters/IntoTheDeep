package dev.cyberarm.minibots.voyager.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Vector2D;

public class VoyagerRobot {
    private final CyberarmEngine engine;
    private final boolean isAutonomous;

    private final DcMotorEx driveFrontLeft, driveFrontRight, driveBackLeft, driveBackRight, lift;
    private final ServoImplEx servo;
    private final IMU imu;

    private Vector2D driveVector = new Vector2D();
    private double driveRotation = 0.0;
    private boolean hardwareFault = false, activehardwareFault = false;
    private double hardwareFaultStartedAt = -1;
    private final double hardwareFaultTimeOut = 500.0;


    public VoyagerRobot(CyberarmEngine engine, boolean isAutonomous) {
        this.engine = engine;
        this.isAutonomous = isAutonomous;

        //-------- MOTORS --------//
        driveFrontLeft = (DcMotorEx) engine.hardwareMap.dcMotor.get("driveFrontLeft");
        driveBackLeft = (DcMotorEx) engine.hardwareMap.dcMotor.get("driveBackLeft");
        driveFrontRight = (DcMotorEx) engine.hardwareMap.dcMotor.get("driveFrontRight");
        driveBackRight = (DcMotorEx) engine.hardwareMap.dcMotor.get("driveBackRight");

        lift = (DcMotorEx) engine.hardwareMap.dcMotor.get("lift");

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        driveFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        driveFrontLeft.setCurrentAlert(1.0, CurrentUnit.AMPS);
        driveBackLeft.setCurrentAlert(1.0, CurrentUnit.AMPS);
        driveFrontRight.setCurrentAlert(1.0, CurrentUnit.AMPS);
        driveBackRight.setCurrentAlert(1.0, CurrentUnit.AMPS);

        lift.setCurrentAlert(1.0, CurrentUnit.AMPS);

        //-------- SERVOS --------//
        servo = (ServoImplEx) engine.hardwareMap.servo.get("servo");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));


        //-------- SENSORS/I2C --------//
        imu = engine.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        //-------- SENSORS/ANALOG --------//

        //-------- LIGHTING --------//
    }

    public void update() {
        protect();

        if (activehardwareFault)
            return;

        updateDriveTrain();

        updateLift();

        updateLighting();
    }

    private void protect() {
        hardwareFault = false;

        if (driveFrontLeft.isOverCurrent())
            hardwareFault = true;
        if (driveBackLeft.isOverCurrent())
            hardwareFault = true;
        if (driveFrontRight.isOverCurrent())
            hardwareFault = true;
        if (driveBackRight.isOverCurrent())
            hardwareFault = true;

        if (lift.isOverCurrent())
            hardwareFault = true;

        if (hardwareFault) {
            // Timeout after 500ms and trigger active hardware fault
            // protection requiring a manual reset.

            if (hardwareFaultStartedAt >= 0 && engine.runTime() - hardwareFaultStartedAt >= hardwareFaultTimeOut) {
                activehardwareFault = true;

                safelyParkHardware();
            } else {
                hardwareFaultStartedAt = engine.runTime();
            }
        } else {
            hardwareFaultStartedAt = -1;
        }
    }

    private void safelyParkHardware() {
        driveFrontLeft.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackRight.setPower(0);

        lift.setPower(0);

        servo.setPwmDisable();
    }

    private void updateDriveTrain() {
        double y = driveVector.y();
        double x = driveVector.x();
        double rx = driveRotation;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftRatio = (rotY + rotX + rx) / denominator;
        double backLeftRatio = (rotY - rotX + rx) / denominator;
        double frontRightRatio = (rotY - rotX - rx) / denominator;
        double backRightRatio = (rotY + rotX - rx) / denominator;

        // FIXME: switch to velocity!
        driveFrontLeft.setPower(1.0 * frontLeftRatio);
        driveBackLeft.setPower(1.0 * backLeftRatio);
        driveFrontRight.setPower(1.0 * frontRightRatio);
        driveBackRight.setPower(1.0 * backRightRatio);
    }

    private void updateLift() {

    }

    private void updateLighting() {

    }
}
