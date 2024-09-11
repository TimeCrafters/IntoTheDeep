package org.timecrafters.scratchpad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ROBOT")
public class ROBOT extends OpMode {
    Servo servo;
    CRServo left, right;
    DcMotor motor;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        right = hardwareMap.crservo.get("right");
        left = hardwareMap.crservo.get("left");
        motor = hardwareMap.dcMotor.get("motor");
//        servo.setPosition(.8);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            right.setPower(1);
            left.setPower(-1);
        } else if (gamepad1.a) {
            right.setPower(-1);
            left.setPower(1);
        } else {
            right.setPower(0);
            left.setPower(0);
        }

        motor.setPower(gamepad1.left_stick_y);

        telemetry.addData("motor position", motor.getCurrentPosition());


    }
}
