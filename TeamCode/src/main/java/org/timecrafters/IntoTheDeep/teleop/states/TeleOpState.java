package org.timecrafters.IntoTheDeep.teleop.states;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.cyberarm.engine.V2.CyberarmState;

public class TeleOpState extends CyberarmState {
    Servo servo;
    CRServo left, right;
    DcMotor motor, frontLeft, frontRight, backLeft, backRight, extension, leftLift, rightLift;
    IMU imu;
    boolean fieldcentrictoggle = true;


    int liftLimit = 1500, extensionLimit = 1000;
    int extensionTargetPos = 0, liftTargetPos = 0;

    @Override
    public void init() {
//        servo = hardwareMap.servo.get("servo");
//        right = hardwareMap.crservo.get("right");
//        left = hardwareMap.crservo.get("left");
//        motor = hardwareMap.dcMotor.get("motor");
        frontLeft = engine.hardwareMap.dcMotor.get("fl");
        frontRight = engine.hardwareMap.dcMotor.get("fr");
        backLeft = engine.hardwareMap.dcMotor.get("bl");
        backRight = engine.hardwareMap.dcMotor.get("br");
        extension = engine.hardwareMap.dcMotor.get("extension");
        leftLift = engine.hardwareMap.dcMotor.get ("leftLift");
        rightLift = engine.hardwareMap.dcMotor.get ("rightLift");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        servo.setPosition(.8);

        // Retrieve the IMU from the hardware map
        imu = engine.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    @Override
    public void exec() {
        //        if (gamepad1.y) {
//            right.setPower(1);
//            left.setPower(-1);
//        } else if (gamepad1.a) {
//            right.setPower(-1);
//            left.setPower(1);
//        } else {
//            right.setPower(0);
//            left.setPower(0);
//        }
//
//        motor.setPower(gamepad1.left_stick_y);

        // EXTENSION
        if (-engine.gamepad1.right_stick_y < 0 && extension.getCurrentPosition() >= 0)
        {
            extension.setPower(-engine.gamepad1.right_stick_y * 1);
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extensionTargetPos = extension.getCurrentPosition();

        } else if (-engine.gamepad1.right_stick_y > 0 && extension.getCurrentPosition() <= extensionLimit) {
            extension.setPower(-engine.gamepad1.right_stick_y * 1);
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extensionTargetPos = extension.getCurrentPosition();

        } else {
            extension.setPower(0.5);
            extension.setTargetPosition(extensionTargetPos);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // LIFT
        if (-engine.gamepad1.left_stick_y < 0 && leftLift.getCurrentPosition() >= 0)
        {
            leftLift.setPower(-engine.gamepad1.left_stick_y *1);
            rightLift.setPower(-engine.gamepad1.left_stick_y *1);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftTargetPos = leftLift.getCurrentPosition();

        } else if (-engine.gamepad1.left_stick_y > 0 && leftLift.getCurrentPosition() <= liftLimit) {
            leftLift.setPower(-engine.gamepad1.left_stick_y *1);
            rightLift.setPower(-engine.gamepad1.left_stick_y *1);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftTargetPos = leftLift.getCurrentPosition();

        } else {
            leftLift.setPower(0.5);
            rightLift.setPower(0.5);
            leftLift.setTargetPosition(liftTargetPos);
            rightLift.setTargetPosition(liftTargetPos);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Tank Drive
        frontLeft.setPower(-engine.gamepad2.left_stick_y);
        frontRight.setPower(-engine.gamepad2.right_stick_y);
        backLeft.setPower(-engine.gamepad2.left_stick_y);
        backRight.setPower(-engine.gamepad2.right_stick_y);

        // Fancy Drive
//        double y = -engine.gamepad2.left_stick_y; // Remember, Y stick value is reversed
//        double x = engine.gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = -engine.gamepad2.right_stick_x;
//
//        double botHeading = fieldcentrictoggle ? 1.57 : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        frontLeft.setPower(frontLeftPower);
//        backLeft.setPower(backLeftPower);
//        frontRight.setPower(frontRightPower);
//        backRight.setPower(backRightPower);
    }

    @Override
    public void buttonDown(Gamepad gamepad, String button) {
        if (engine.gamepad2 == gamepad) {
            switch (button) {
                case "guide": {
                    imu.resetYaw();
                    break;
                }
                case "start": {
                    fieldcentrictoggle = !fieldcentrictoggle;
                    break;
                }
            }
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("extension", extension.getCurrentPosition());
        engine.telemetry.addData("leftLift", leftLift.getCurrentPosition());
        engine.telemetry.addData("rightLift", rightLift.getCurrentPosition());
        engine.telemetry.addData("extensionTargetPos", extensionTargetPos);
        engine.telemetry.addData("liftTargetPos", liftTargetPos);
        engine.telemetry.addData("IMU",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        engine.telemetry.addData("fieldcentrictoggle", fieldcentrictoggle);
    }
}
