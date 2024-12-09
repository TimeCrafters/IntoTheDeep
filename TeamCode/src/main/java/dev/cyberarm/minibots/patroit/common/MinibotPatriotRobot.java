package dev.cyberarm.minibots.patroit.common;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.engine.V2.Vector2D;

public class MinibotPatriotRobot {
    public final DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    public final DcMotorEx extension, leftLift, rightLift;
    public final ServoImplEx intakeClaw, depositorClaw, depositorLeft, depositorRight;
    public final CRServoImplEx intakeLeftDiff, intakeRightDiff;
    public final IMU imu;
    public boolean isPreciseDrivetrainVelocity = false;
    private final CyberarmEngine engine;
    private final boolean isAutonomous;
    private final SparkFunOTOS odometry;
    private final OctoQuad octoquad;
    private OctoQuad.EncoderDataBlock octoData = new OctoQuad.EncoderDataBlock();
    private Pose2D position = new Pose2D();
    private Pose2D targetPosition = new Pose2D();
    private State lastState, state = State.COMPACT, requestedState;
    private boolean isTransferring = false;

    public enum OctoEncoder {
        INTAKE_LEFT_DIFF,
        INTAKE_RIGHT_DIFF,
        DEPOSITOR_LEFT_DIFF,
        DEPOSITOR_RIGHT_DIFF
    }

    public enum State {
        COMPACT, // Stow all hands and limbs
        SAMPLE_COLLECT, // Extend extension, stow lift, pre position and open depositor claw, position and open intake claw
        SAMPLE_DEPOSIT, // Ensure transfer complete, position depositor claw, lift the lift up to the high basket
        SPECIMEN_COLLECT, // Stow intake claw, stow extension, stow and position lift, position depositor claw
        SPECIMEN_DEPOSIT, // Stow intake claw, stow extension, raise lift, position depositor claw
        TRANSFER, // [AUTOMATIC] Stow lift, pre position and open depositor claw, stow intake claw, stow extension, close depositor claw, open intake claw
        ACCENT_LEVEL_ONE, // Safely stow all hands and limbs, position depositor claw, lift the lift above the bar
        ACCENT_LEVEL_TWO, // NO OP
        ACCENT_LEVEL_THREE, // NO OP
        PANIC // Stop and shutdown all actuators as quickly as possible
    }

    public MinibotPatriotRobot(CyberarmEngine engine, boolean isAutonomous) {
        this.engine = engine;
        this.isAutonomous = isAutonomous;

        // ODOMETRY
        odometry = engine.hardwareMap.get(SparkFunOTOS.class, "otos");

        // IMU
        imu = engine.hardwareMap.get(IMU.class, "imu_ex");

        // OCTOQUAD / ENCODERS
        octoquad = engine.hardwareMap.get(OctoQuad.class, "octoquad");

        // MOTORS
        extension = (DcMotorEx) engine.hardwareMap.dcMotor.get("extension");
        leftLift = (DcMotorEx) engine.hardwareMap.dcMotor.get("leftLift");
        rightLift = (DcMotorEx) engine.hardwareMap.dcMotor.get("rightLift");

        frontLeftDrive = (DcMotorEx) engine.hardwareMap.dcMotor.get("fl");
        frontRightDrive = (DcMotorEx) engine.hardwareMap.dcMotor.get("fr");
        backRightDrive = (DcMotorEx) engine.hardwareMap.dcMotor.get("br");
        backLeftDrive = (DcMotorEx) engine.hardwareMap.dcMotor.get("bl");

        // SERVOS
        intakeClaw = (ServoImplEx) engine.hardwareMap.servo.get("theClaw");
        depositorClaw = (ServoImplEx) engine.hardwareMap.servo.get("depo claw");
        depositorLeft = (ServoImplEx) engine.hardwareMap.servo.get("depo left");
        depositorRight = (ServoImplEx) engine.hardwareMap.servo.get("depo right");

        // CONTINUOUS SERVOS
        intakeLeftDiff = (CRServoImplEx) engine.hardwareMap.crservo.get("leftDiff");
        intakeRightDiff = (CRServoImplEx) engine.hardwareMap.crservo.get("rightDiff");

        setup();
    }

    private void setup() {
        // ODOMETRY
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(1.0);
        odometry.setAngularScalar(1.0);

        // Only reset sensor for autonomous
        if (!isAutonomous) {
            odometry.calibrateImu();
            odometry.resetTracking();

            // FIXME: calculate sensor offset
            odometry.setOffset(new Pose2D(0, 0, 0));
            // FIXME: determine initial pose for autonomous for absolute field coordinate positioning
            odometry.setPosition(new Pose2D(0, 0, 0));
        }

        // IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        // OCTOQUAD / ENCODERS
        octoquad.setSingleEncoderDirection(OctoEncoder.INTAKE_LEFT_DIFF.ordinal(), OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(OctoEncoder.INTAKE_RIGHT_DIFF.ordinal(), OctoQuad.EncoderDirection.REVERSE);

        // MOTORS
        // only reset encoders when starting autonomous, preserves positions for TeleOp automatics)
        if (isAutonomous) {
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SERVOS
        depositorLeft.setDirection(Servo.Direction.FORWARD);
        depositorRight.setDirection(Servo.Direction.REVERSE);

        // CONTINUOUS SERVOS
        intakeLeftDiff.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRightDiff.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        if (state == State.PANIC) {
            handlePanic();
            return;
        }

        // ODOMETRY
        position = odometry.getPosition();

        // OCTOQUAD / ENCODERS
        octoquad.readAllEncoderData(octoData);

        // Handle requested state
        if (requestedState != null && !isTransferring) {
            this.lastState = state;
            this.state = requestedState;
            requestedState = null;
        }

        handleExtension();
        handleIntakeClaw();
        handleDepositorClaw();
        handleLift();

        handleDrivetrain();
    }

    public void teardown() {

    }

    public void telemetry() {
        engine.telemetry.addLine("Patriot Robot");
        engine.telemetry.addData("State", "%s", state.name());
        engine.telemetry.addData("Position", "X: %.3f\", Y: %.3f\", H: %.3f\"", position.x, position.y, position.h);
        engine.telemetry.addLine("");
        engine.telemetry.addLine("MOTORS");
        engine.telemetry.addData("Extension", "Position: %d, Power: %.3f, Velocity: %.3f, Current: %.3fA", extension.getCurrentPosition(), extension.getPower(), extension.getVelocity(), extension.getCurrent(CurrentUnit.AMPS));
        engine.telemetry.addData("Left Lift", "Position: %d, Power: %.3f, Velocity: %.3f, Current: %.3fA", leftLift.getCurrentPosition(), leftLift.getPower(), leftLift.getVelocity(), leftLift.getCurrent(CurrentUnit.AMPS));
        engine.telemetry.addData("Right Lift", "Position: %d, Power: %.3f, Velocity: %.3f, Current: %.3fA", rightLift.getCurrentPosition(), rightLift.getPower(), rightLift.getVelocity(), rightLift.getCurrent(CurrentUnit.AMPS));
        engine.telemetry.addLine("");
        engine.telemetry.addData("Front Left Drive", "Position: %d, Power: %.3f, Velocity: %.3f, Current: %.3fA", frontLeftDrive.getCurrentPosition(), frontLeftDrive.getPower(), frontLeftDrive.getVelocity(), frontLeftDrive.getCurrent(CurrentUnit.AMPS));
        engine.telemetry.addData("Front Right Drive", "Position: %d, Power: %.3f, Velocity: %.3f, Current: %.3fA", frontRightDrive.getCurrentPosition(), frontRightDrive.getPower(), frontRightDrive.getVelocity(), frontRightDrive.getCurrent(CurrentUnit.AMPS));
        engine.telemetry.addData("Back Left Drive", "Position: %d, Power: %.3f, Velocity: %.3f, Current: %.3fA", backLeftDrive.getCurrentPosition(), backLeftDrive.getPower(), backLeftDrive.getVelocity(), backLeftDrive.getCurrent(CurrentUnit.AMPS));
        engine.telemetry.addData("Back Right Drive", "Position: %d, Power: %.3f, Velocity: %.3f, Current: %.3fA", backRightDrive.getCurrentPosition(), backRightDrive.getPower(), backRightDrive.getVelocity(), backRightDrive.getCurrent(CurrentUnit.AMPS));
        engine.telemetry.addLine("");
        engine.telemetry.addLine("SERVOS");
        engine.telemetry.addData("Intake Claw", "Position: %.3f", intakeClaw.getPosition());
        engine.telemetry.addData("Depositor Claw", "Position: %.3f", depositorClaw.getPosition());
        engine.telemetry.addData("Depositor Left", "Position: %.3f", depositorLeft.getPosition());
        engine.telemetry.addData("Depositor Right", "Position: %.3f", depositorRight.getPosition());
        engine.telemetry.addLine("");
        engine.telemetry.addLine("CONTINUOUS SERVOS");
        engine.telemetry.addData("Intake Left Diff", "Position: %d, Power: %.3f, Velocity: %.3f", getOctoPosition(OctoEncoder.INTAKE_LEFT_DIFF), intakeLeftDiff.getPower(), getOctoVelocity(OctoEncoder.INTAKE_LEFT_DIFF));
        engine.telemetry.addData("Intake Right Diff", "Position: %d, Power: %.3f, Velocity: %.3f", getOctoPosition(OctoEncoder.INTAKE_RIGHT_DIFF), intakeRightDiff.getPower(), getOctoVelocity(OctoEncoder.INTAKE_RIGHT_DIFF));
    }

    public int getOctoPosition(OctoEncoder id) {
        return octoData.positions[id.ordinal()];
    }

    public int getOctoVelocity(OctoEncoder id) {
        return octoData.velocities[id.ordinal()];
    }

    public Pose2D getPosition() {
        return position;
    }

    public Pose2D getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(Pose2D position) {
        this.targetPosition = position;
    }

    public void drivetrainRobotCentric(double forward, double right, double rotate) {
        // double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        //  double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        //  double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(rotate), 1);
        double frontLeftPower = (forward + right + rotate) / denominator;
        double backLeftPower = (forward - right + rotate) / denominator;
        double frontRightPower = (forward - right - rotate) / denominator;
        double backRightPower = (forward + right - rotate) / denominator;

        // FIXME: Use velocity
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    public void drivetrainFieldCentric(double forward, double right, double rotate) {
        // double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        // double x = gamepad1.left_stick_x;
        // double rx = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = right * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = right * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double frontLeftPower = (rotY + rotX + rotate) / denominator;
        double backLeftPower = (rotY - rotX + rotate) / denominator;
        double frontRightPower = (rotY - rotX - rotate) / denominator;
        double backRightPower = (rotY + rotX - rotate) / denominator;

        // FIXME: Use velocity
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    public State getState() {
        return state;
    }

    public void requestState(State state) {
        this.requestedState = state;
    }

    public void togglePanic() {
        if (state == State.PANIC) {
            state = lastState;
        } else {
            lastState = state;
            state = State.PANIC;
        }
    }

    private void handlePanic() {
        // MOTORS
        extension.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        // SERVOS
        intakeClaw.setPwmDisable();
        depositorClaw.setPwmDisable();
        depositorLeft.setPwmDisable();
        depositorRight.setPwmDisable();

        // CONTINUOUS SERVOS
        intakeLeftDiff.setPwmDisable();
        intakeRightDiff.setPwmDisable();
    }

    private void handleExtension() {}

    private void handleIntakeClaw() {}
    private void handleDepositorClaw() {}
    private void handleLift() {}
    private void handleDrivetrain() {
        if (!isAutonomous)
            return;

        // NOTE: May need to swap position and targetPosition vectors around, may result in inverted vector.
        Vector2D targetVector = (new Vector2D(position.x, position.y).minus(new Vector2D(targetPosition.x, targetPosition.y)).normalize());
        // NOTE: May need to swap position heading and targetPosition heading, my result in inverted angle difference.
        double angleDiff = Utilities.angleDiff(position.h, targetPosition.h);

        drivetrainFieldCentric(targetVector.y(), targetVector.x(), angleDiff / 360.0);
    }
}
