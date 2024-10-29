package org.timecrafters.IntoTheDeep.teleop.states;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.IntoTheDeep.common.Patriot;

import dev.cyberarm.engine.V2.CyberarmState;

public class PatriotTeleOpState extends CyberarmState {

    private final Patriot robot;
    private double drivePower = 1;
    private boolean lbsVar;

    public PatriotTeleOpState(Patriot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {

        // reset orientation, and move camera when reset to see...
        if (engine.gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

        // toggle to enter slow or fast mode
        boolean lbs = engine.gamepad1.left_stick_button;
        if (lbs && !lbsVar) {
            if (drivePower == 1) {
                drivePower = 0.5;
            } else {
                drivePower = 1;
            }
        }
        lbsVar = lbs;

        // math and variables for field oriented drive:
        // the variables that are used for the joystick math later on
        double y = -engine.gamepad1.left_stick_y;
        double x = -engine.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -engine.gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // angle math to make things field oriented
        double heading = (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // setting each power determined previously from the math above
        // as well as multiplying it by a drive power that can be changed.
        robot.bl.setPower(backLeftPower * drivePower);
        robot.br.setPower(backRightPower * drivePower);
        robot.fl.setPower(frontLeftPower * drivePower);
        robot.fr.setPower(frontRightPower * drivePower);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("imu yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("imu pitch", -robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        engine.telemetry.addData("imu roll", -robot.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
    }
}