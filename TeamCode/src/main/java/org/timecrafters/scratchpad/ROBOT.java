package org.timecrafters.scratchpad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ROBOT")
public class ROBOT extends OpMode {
    Servo servo;
    CRServo left, right;
    DcMotor motor, frontLeft, frontRight, backLeft, backRight, extension;

    @Override
    public void init() {
//        servo = hardwareMap.servo.get("servo");
//        right = hardwareMap.crservo.get("right");
//        left = hardwareMap.crservo.get("left");
//        motor = hardwareMap.dcMotor.get("motor");
//        frontLeft = hardwareMap.dcMotor.get("frontLeft");
//        frontRight = hardwareMap.dcMotor.get("frontRight");
//        backLeft = hardwareMap.dcMotor.get("backLeft");
//        backRight = hardwareMap.dcMotor.get("backRight");
        extension = hardwareMap.dcMotor.get("extension");

//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//        servo.setPosition(.8);
    }

    @Override
    public void loop() {
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
        extension.setPower(gamepad1.right_stick_y *1);

//        frontLeft.setPower(gamepad2.left_stick_y);
//        frontRight.setPower(gamepad2.right_stick_y);
//        backLeft.setPower(gamepad2.left_stick_y);
//        backRight.setPower(gamepad2.right_stick_y);

//        double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = -gamepad2.right_stick_x;
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        frontLeft.setPower(frontLeftPower);
//        backLeft.setPower(backLeftPower);
//        frontRight.setPower(frontRightPower);
//        backRight.setPower(backRightPower);
//
//
//        telemetry.addData("motor position", motor.getCurrentPosition());


    }
}
