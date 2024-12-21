package dev.cyberarm.minibots.patriot.common;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.engine.V2.Vector2D;

public class MinibotPatriotRobot {
    private static MinibotPatriotRobot instance;

    // Named indices of Octoquad encoders
    public enum OctoEncoder {
        INTAKE_LEFT_DIFF,
        INTAKE_RIGHT_DIFF,
        DEPOSITOR_LEFT_DIFF,
        DEPOSITOR_RIGHT_DIFF
    }

    // State of robot for automatics
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

    // Various hardware states for automatics to coordinate
    public enum HardwareState {
        INTAKE_CLAW_OPEN,
        INTAKE_CLAW_CLOSED,
        INTAKE_DIFFERENTIAL_COLLECT,
        INTAKE_DIFFERENTIAL_STOW,
        EXTENSION_STOW,
        EXTENSION_OUT,
        DEPOSITOR_CLAW_OPEN,
        DEPOSITOR_CLAW_CLOSED,
        DEPOSITOR_ARM_DEPOSIT,
        DEPOSITOR_ARM_STOW,
        LIFT_STOW,
        LIFT_LOW_BASKET,
        LIFT_HIGH_BASKET,
        LIFT_LOW_RUNG,
        LIFT_HIGH_RUNG,
        LIFT_SPECIMEN,
        DRIVETRAIN_IDLE,
        DRIVETRAIN_MOVING,
    }

    public final DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    public final DcMotorEx extension, leftLift, rightLift;
    public final ServoImplEx intakeClaw, depositorClaw, depositorLeft, depositorRight;
    public final CRServoImplEx intakeLeftDiff, intakeRightDiff;
    public final IMU imu;
    public TimeCraftersConfiguration config;
    public boolean isPreciseDrivetrainVelocity = false;
    private final CyberarmEngine engine;
    private boolean isAutonomous;
    private final SparkFunOTOS odometry;
    private final OctoQuad octoquad;
    private OctoQuad.EncoderDataBlock octoData = new OctoQuad.EncoderDataBlock();
    private Pose2D odometrySensorOffset = new Pose2D();
    private Pose2D position = new Pose2D();
    private Pose2D targetPosition = new Pose2D();
    private State lastState, state = State.COMPACT, requestedState;
    private boolean isTransferring = false;
    private int drivetrainVelocity = 1500, drivetrainPreciseVelocity = 1500, drivetrainCoarseVelocity = 1500;
    private int extensionVelocity = 1500, extensionPreciseVelocity = 1500, extensionCoarseVelocity = 1500;
    private int liftVelocity = 1500, liftPreciseVelocity = 1500, liftCoarseVelocity = 1500;
    private int drivetrainTicksPerRevolution;
    private double drivetrainGearRatio, drivetrainWheelDiameterMM;
    private int extensionTicksPerRevolution;
    private double extensionGearRatio, extensionWheelDiameterMM;
    private int liftTicksPerRevolution;
    private double liftGearRatio, liftWheelDiameterMM;
    private double intakeClawOpenPosition = 0.4, intakeClawClosedPosition = 1.0;
    private int intakeDifferentialStowPosition = 0, intakeDifferentialCollectPosition = 5800, intakeDifferentialPosition = 0, intakeLeftDifferential = 0, intakeRightDifferential = 0;
    private double depositorClawOpenPosition = 0.75, depositorClawClosedPosition = 1.0;
    private int depositorDifferentialUprightPosition = 0, depositorDifferentialInvertedPosition = 0, depositorPosition = 0, depositorDifferential = 0;
    private double depositorArmStowPosition = 0.04, depositorArmDepositPosition = 1.0;
    private final PIDFController intakeLeftController, intakeRightController;


    public static MinibotPatriotRobot getInstance() {
        return instance;
    }

    public MinibotPatriotRobot(CyberarmEngine engine, boolean isAutonomous) {
        this.engine = engine;
        this.isAutonomous = isAutonomous;

        MinibotPatriotRobot.instance = this;

        // TimeCrafters Configuration
        this.config = new TimeCraftersConfiguration("MinibotPatriot");

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
        intakeLeftDiff = (CRServoImplEx) engine.hardwareMap.crservo.get("leftIntakeDiff");
        intakeRightDiff = (CRServoImplEx) engine.hardwareMap.crservo.get("rightIntakeDiff");

        intakeLeftController = new PIDFController(intakeLeftDiff);
        intakeRightController = new PIDFController(intakeRightDiff);

        loadConfig();

        setup();
    }

    private void loadConfig() {
        final String groupName = "Robot";
        final String actionName = "Hardware";

        // ODOMETRY
        final double odometryOffsetXInches = config.variable(groupName, actionName, "odometry_offset_x_inches").value();
        final double odometryOffsetYInches = config.variable(groupName, actionName, "odometry_offset_y_inches").value();
        final double odometryOffsetHeadingDegrees = config.variable(groupName, actionName, "odometry_offset_heading_degrees").value();
        odometrySensorOffset.set(new Pose2D(
                odometryOffsetXInches,
                odometryOffsetYInches,
                odometryOffsetHeadingDegrees
        ));

        // DRIVETRAIN
        drivetrainGearRatio = config.variable(groupName, actionName, "drivetrain_gear_ratio").value();
        drivetrainTicksPerRevolution = config.variable(groupName, actionName, "drivetrain_ticks_per_revolution").value();
        drivetrainWheelDiameterMM = config.variable(groupName, actionName, "drivetrain_wheel_diameter_mm").value();
        final double drivetrainCoarseVelocityInches = config.variable(groupName, actionName, "drivetrain_coarse_velocity_inches").value();
        final double drivetrainPreciseVelocityInches = config.variable(groupName, actionName, "drivetrain_precise_velocity_inches").value();

        drivetrainCoarseVelocity = Utilities.unitToTicks(drivetrainTicksPerRevolution, drivetrainGearRatio, drivetrainWheelDiameterMM, DistanceUnit.INCH, drivetrainCoarseVelocityInches);
        drivetrainPreciseVelocity = Utilities.unitToTicks(drivetrainTicksPerRevolution, drivetrainGearRatio, drivetrainWheelDiameterMM, DistanceUnit.INCH, drivetrainPreciseVelocityInches);

        drivetrainVelocity = drivetrainCoarseVelocity;

        // EXTENSION
        extensionGearRatio = config.variable(groupName, actionName, "extension_gear_ratio").value();
        extensionTicksPerRevolution = config.variable(groupName, actionName, "extension_ticks_per_revolution").value();
        extensionWheelDiameterMM = config.variable(groupName, actionName, "extension_wheel_diameter_mm").value();
        final double extensionCoarseVelocityInches = config.variable(groupName, actionName, "extension_coarse_velocity_inches").value();
        final double extensionPreciseVelocityInches = config.variable(groupName, actionName, "extension_precise_velocity_inches").value();

        extensionCoarseVelocity = Utilities.unitToTicks(extensionTicksPerRevolution, extensionGearRatio, extensionWheelDiameterMM, DistanceUnit.INCH, extensionCoarseVelocityInches);
        extensionPreciseVelocity = Utilities.unitToTicks(extensionTicksPerRevolution, extensionGearRatio, extensionWheelDiameterMM, DistanceUnit.INCH, extensionPreciseVelocityInches);

        extensionVelocity = extensionCoarseVelocity;

        // LIFT
        liftGearRatio = config.variable(groupName, actionName, "lift_gear_ratio").value();
        liftTicksPerRevolution = config.variable(groupName, actionName, "lift_ticks_per_revolution").value();
        liftWheelDiameterMM = config.variable(groupName, actionName, "lift_wheel_diameter_mm").value();
        final double liftCoarseVelocityInches = config.variable(groupName, actionName, "lift_coarse_velocity_inches").value();
        final double liftPreciseVelocityInches = config.variable(groupName, actionName, "lift_precise_velocity_inches").value();

        liftCoarseVelocity = Utilities.unitToTicks(liftTicksPerRevolution, liftGearRatio, liftWheelDiameterMM, DistanceUnit.INCH, liftCoarseVelocityInches);
        liftPreciseVelocity = Utilities.unitToTicks(liftTicksPerRevolution, liftGearRatio, liftWheelDiameterMM, DistanceUnit.INCH, liftPreciseVelocityInches);

        liftVelocity = liftCoarseVelocity;

        // INTAKE CLAW
        intakeClawOpenPosition = config.variable(groupName, actionName, "intake_claw_open_position").value();
        intakeClawClosedPosition = config.variable(groupName, actionName, "intake_claw_closed_position").value();

        intakeDifferentialStowPosition = config.variable(groupName, actionName, "intake_differential_stow_position").value();
        intakeDifferentialCollectPosition = config.variable(groupName, actionName, "intake_differential_collect_position").value();

        // DEPOSITOR CLAW
        depositorClawOpenPosition = config.variable(groupName, actionName, "depositor_claw_open_position").value();
        depositorClawClosedPosition = config.variable(groupName, actionName, "depositor_claw_closed_position").value();

        depositorDifferentialUprightPosition = config.variable(groupName, actionName, "depositor_differential_upright_position").value();
        depositorDifferentialInvertedPosition = config.variable(groupName, actionName, "depositor_differential_inverted_position").value();

        depositorArmStowPosition = config.variable(groupName, actionName, "depositor_arm_stow_position").value();
        depositorArmDepositPosition = config.variable(groupName, actionName, "depositor_arm_deposit_position").value();
    }

    private void setup() {
        // Lynx Modules
        Utilities.hubsBulkReadMode(engine.hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        // ODOMETRY
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(1.0);
        odometry.setAngularScalar(1.0);

        // Only reset sensor for autonomous
        if (!isAutonomous) {
            odometry.calibrateImu(255, false);
            odometry.resetTracking();

            odometry.setOffset(odometrySensorOffset);
            odometry.setPosition(new Pose2D(0, 0, 0));
        }

        // IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        if (isAutonomous)
            imu.resetYaw();

        // OCTOQUAD / ENCODERS
        octoquad.setSingleEncoderDirection(OctoEncoder.INTAKE_LEFT_DIFF.ordinal(), OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(OctoEncoder.INTAKE_RIGHT_DIFF.ordinal(), OctoQuad.EncoderDirection.REVERSE);

        if (isAutonomous)
            octoquad.resetEverything();

        // MOTORS
        // only reset encoders when starting autonomous, preserves positions for TeleOp automatics)
        if (isAutonomous) {
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            extension.setTargetPosition(0);
            leftLift.setTargetPosition(0);
            rightLift.setTargetPosition(0);

            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

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

        if (isAutonomous) {
            positionClaw(HardwareState.INTAKE_CLAW_CLOSED);
            positionClaw(HardwareState.DEPOSITOR_CLAW_OPEN);

            positionDepositorArm(HardwareState.DEPOSITOR_ARM_STOW);
        }

        // CONTINUOUS SERVOS
        intakeLeftDiff.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRightDiff.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        if (state == State.PANIC) {
            handlePanic();
            return;
        }

        // Lynx Modules
        Utilities.hubsClearBulkReadCache(engine.hardwareMap);

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
        // Stop drive wheels
        drivetrainFieldCentric(0, 0, 0);
    }

    public void telemetry() {
        engine.telemetry.addLine("Patriot Robot");
        engine.telemetry.addData("State", "%s", state.name());
        engine.telemetry.addData("Requested State", "%s", (requestedState == null ? "-"  : requestedState.name()));
        engine.telemetry.addData("Position", "X: %.3f\", Y: %.3f\", H: %.3f°, IMU: %.3f°", position.x, position.y, position.h, Utilities.facing(imu));
        engine.telemetry.addData("Target Position", "X: %.3f\", Y: %.3f\", H: %.3f°", targetPosition.x, targetPosition.y, targetPosition.h);
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
        engine.telemetry.addData("Intake Left Diff", "Position: %d, Power: %.3f, Velocity: %d", getOctoPosition(OctoEncoder.INTAKE_LEFT_DIFF), intakeLeftDiff.getPower(), getOctoVelocity(OctoEncoder.INTAKE_LEFT_DIFF));
        engine.telemetry.addData("Intake Right Diff", "Position: %d, Power: %.3f, Velocity: %d", getOctoPosition(OctoEncoder.INTAKE_RIGHT_DIFF), intakeRightDiff.getPower(), getOctoVelocity(OctoEncoder.INTAKE_RIGHT_DIFF));
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

    public void setInitialPosition(Pose2D position) {
        this.odometry.setPosition(position);
    }

    public void setTeleOp() {
        this.isAutonomous = false;
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

        frontLeftDrive.setVelocity(frontLeftPower * drivetrainVelocity);
        backLeftDrive.setVelocity(backLeftPower * drivetrainVelocity);
        frontRightDrive.setVelocity(frontRightPower * drivetrainVelocity);
        backRightDrive.setVelocity(backRightPower * drivetrainVelocity);
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

        frontLeftDrive.setVelocity(frontLeftPower * drivetrainVelocity);
        backLeftDrive.setVelocity(backLeftPower * drivetrainVelocity);
        frontRightDrive.setVelocity(frontRightPower * drivetrainVelocity);
        backRightDrive.setVelocity(backRightPower * drivetrainVelocity);
    }

    public void positionClaw(HardwareState state) {
        switch (state) {
            case DEPOSITOR_CLAW_CLOSED: {
                depositorClaw.setPosition(depositorClawClosedPosition);
                break;
            }
            case DEPOSITOR_CLAW_OPEN: {
                depositorClaw.setPosition(depositorClawOpenPosition);
                break;
            }
            case INTAKE_CLAW_CLOSED: {
                intakeClaw.setPosition(intakeClawClosedPosition);
                break;
            }
            case INTAKE_CLAW_OPEN: {
                intakeClaw.setPosition(intakeClawOpenPosition);
                break;
            }
        }
    }

    public void positionDepositorArm(HardwareState state) {
        switch (state) {
            case DEPOSITOR_ARM_STOW: {
                depositorLeft.setPosition(depositorArmStowPosition);
                depositorRight.setPosition(depositorArmStowPosition);
                break;
            }
            case DEPOSITOR_ARM_DEPOSIT: {
                depositorLeft.setPosition(depositorArmDepositPosition);
                depositorRight.setPosition(depositorArmDepositPosition);
                break;
            }
        }
    }

    public void setLiftReach(double targetInches, double toleranceInches) {
        final int targetPosition = Utilities.unitToTicks(
                liftTicksPerRevolution,
                liftGearRatio,
                liftWheelDiameterMM,
                DistanceUnit.INCH,
                targetInches);
        final int targetTolerance = Utilities.unitToTicks(
                liftTicksPerRevolution,
                liftGearRatio,
                liftWheelDiameterMM,
                DistanceUnit.INCH,
                toleranceInches);

        leftLift.setTargetPosition(targetPosition);
        rightLift.setTargetPosition(targetPosition);

        leftLift.setTargetPositionTolerance(targetTolerance);
        rightLift.setTargetPositionTolerance(targetTolerance);
    }

    // FIXME: Use real numbers
    public void positionExtension(HardwareState state) {
        switch (state) {
            case EXTENSION_OUT: {
                extension.setTargetPosition(800);
            }
            case EXTENSION_STOW: {
                extension.setTargetPosition(0);
            }
        }
    }

    // FIXME: Use real numbers
    public void positionLift(HardwareState state) {
        switch (state) {
            case LIFT_STOW: {
                leftLift.setTargetPosition(0);
                rightLift.setTargetPosition(0);
            }
            case LIFT_HIGH_BASKET: {
                leftLift.setTargetPosition(0);
                rightLift.setTargetPosition(0);
            }
            case LIFT_LOW_BASKET: {
                leftLift.setTargetPosition(0);
                rightLift.setTargetPosition(0);
            }
            case LIFT_HIGH_RUNG: {
                leftLift.setTargetPosition(0);
                rightLift.setTargetPosition(0);
            }
            case LIFT_LOW_RUNG: {
                leftLift.setTargetPosition(0);
                rightLift.setTargetPosition(0);
            }
            case LIFT_SPECIMEN: {
                leftLift.setTargetPosition(0);
                rightLift.setTargetPosition(0);
            }
        }
    }

    public void setExtensionReach(double targetInches, double toleranceInches) {
        final int targetPosition = Utilities.unitToTicks(
                extensionTicksPerRevolution,
                extensionGearRatio,
                extensionWheelDiameterMM,
                DistanceUnit.INCH,
                targetInches);
        final int targetTolerance = Utilities.unitToTicks(
                extensionTicksPerRevolution,
                extensionGearRatio,
                extensionWheelDiameterMM,
                DistanceUnit.INCH,
                toleranceInches);

        extension.setTargetPosition(targetPosition);
        extension.setTargetPositionTolerance(targetTolerance);
    }

    public void positionIntakeDifferential(HardwareState state, int differential) {
        if (state == HardwareState.INTAKE_DIFFERENTIAL_COLLECT)
            intakeDifferentialPosition = intakeDifferentialCollectPosition;
        if (state == HardwareState.INTAKE_DIFFERENTIAL_STOW)
            intakeDifferentialPosition = intakeDifferentialStowPosition;

        intakeLeftDifferential = differential;
        intakeRightDifferential = -differential;
    }

    public boolean intakeDifferentialAtPosition() {
        return (Utilities.isBetween(
                    getOctoPosition(OctoEncoder.INTAKE_LEFT_DIFF),
                    intakeDifferentialPosition + intakeLeftDifferential - 50,
                    intakeDifferentialPosition + intakeLeftDifferential + 50) &&
                Utilities.isBetween(
                    getOctoPosition(OctoEncoder.INTAKE_RIGHT_DIFF),
                    intakeDifferentialPosition + intakeRightDifferential - 50,
                    intakeDifferentialPosition + intakeRightDifferential + 50));
    }



    public State getState() {
        return state;
    }

    public State getRequestedState() {
        return this.requestedState;
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
    private void handleIntakeClaw() {
        final int extensionPosition = extension.getCurrentPosition();
        int leftDiffTarget = intakeDifferentialPosition + intakeLeftDifferential;
        int rightDiffTarget = intakeDifferentialPosition + intakeRightDifferential;

        // Stow collector when not safe to be down
        // FIXME: Don't use a static number...
        if (extensionPosition <= 300) {
            leftDiffTarget = 0;
            rightDiffTarget = 0;
        }

        intakeLeftController.update(
                getOctoPosition(OctoEncoder.INTAKE_LEFT_DIFF),
                leftDiffTarget);
        intakeRightController.update(
                getOctoPosition(OctoEncoder.INTAKE_RIGHT_DIFF),
                rightDiffTarget);
    }
    private void handleDepositorClaw() {}
    private void handleLift() {}
    private void handleDrivetrain() {
        if (!isAutonomous)
            return;

        // NOTE: May need to swap position and targetPosition vectors around, may result in inverted vector.
        Vector2D targetVector = (new Vector2D(position.x, position.y).minus(new Vector2D(targetPosition.x, targetPosition.y)).normalize());
        // NOTE: May need to swap position heading and targetPosition heading, my result in inverted angle difference.
        double angleDiff = Utilities.angleDiff(Utilities.facing(position.h), Utilities.facing(targetPosition.h));

        drivetrainFieldCentric(-targetVector.y(), targetVector.x(), angleDiff / 180.0);
    }
}
