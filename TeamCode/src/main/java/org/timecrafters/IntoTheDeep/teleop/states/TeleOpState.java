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

    @Override
    public void init() {
//        servo = hardwareMap.servo.get("servo");
//        right = hardwareMap.crservo.get("right");
//        left = hardwareMap.crservo.get("left");
//        motor = hardwareMap.dcMotor.get("motor");
        frontLeft = engine.hardwareMap.dcMotor.get("frontLeft");
        frontRight = engine.hardwareMap.dcMotor.get("frontRight");
        backLeft = engine.hardwareMap.dcMotor.get("backLeft");
        backRight = engine.hardwareMap.dcMotor.get("backRight");
        extension = engine.hardwareMap.dcMotor.get("extension");
        leftLift = engine.hardwareMap.dcMotor.get ("leftLift");
        rightLift = engine.hardwareMap.dcMotor.get ("rightLift");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

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

        extension.setPower(-engine.gamepad1.right_stick_y *1);
        leftLift.setPower(engine.gamepad1.left_stick_y *1);
        rightLift.setPower(engine.gamepad1.left_stick_y *1);

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
        engine.telemetry.addData("IMU",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        engine.telemetry.addData("fieldcentrictoggle", fieldcentrictoggle);
    }
}
