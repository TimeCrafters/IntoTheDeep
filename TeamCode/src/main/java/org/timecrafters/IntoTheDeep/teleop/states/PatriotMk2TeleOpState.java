package org.timecrafters.IntoTheDeep.teleop.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.IntoTheDeep.common.PatriotMk2;

import dev.cyberarm.engine.V2.CyberarmState;

public class PatriotMk2TeleOpState extends CyberarmState {

    private final PatriotMk2 robot;
    private double drivePower = 1;
    private double maxVelocity = 2700 * 0.85;
    private boolean lbsVar;

    private double recordVelo;
    private double testPower = 0;
    public boolean depositManualControl = false;
    public double depositorArmPower = 2;
//    public int intakeTarget = 0;
    public int depositTarget = 0;
//
//    public int leftDifTarget = 0;
//    public int rightDifTarget = 0;
    public double maxExtendoVelo = 0;
    private double rx;
    private boolean headingLock;
    private double headingTarget;




    public PatriotMk2TeleOpState(PatriotMk2 robot) {
        this.robot = robot;
    }

    public void DriveTrainControl() {

        // -------------------------------------------------------------------------------------------------------------- Player 1 (Wheels)

        if (engine.gamepad1.dpad_down) {
            headingTarget = Math.toRadians(0);
        } else if (engine.gamepad1.dpad_up) {
            headingTarget = Math.toRadians(180);
        } else if (engine.gamepad1.dpad_left) {
            headingTarget = Math.toRadians(90);
        } else if (engine.gamepad1.dpad_right) {
            headingTarget = Math.toRadians(-90);
        }

        if (headingLock){
            rx = robot.HeadingPIDControl(headingTarget, robot.imu.getRobotYawPitchRollAngles().getYaw());
        } else {
            rx = engine.gamepad1.right_stick_x;
        }

        // reset orientation, and move camera when reset to see...
        if (engine.gamepad1.b) {
            robot.imu.resetYaw();
        }

        // toggle to enter slow or fast mode
        boolean a = engine.gamepad1.a;
        if (a && !lbsVar) {
            if (drivePower == 1) {
                drivePower = 0.5;
            } else {
                drivePower = 1;
            }
        }
        lbsVar = a;

        if (engine.gamepad1.left_trigger != 0){
            testPower = 100000;
        } else {
            testPower = 0;
        }

        // math and variables for field oriented drive:
        // the variables that are used for the joystick math later on
        double y = -engine.gamepad1.left_stick_y;
        double x = engine.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // angle math to make things field oriented
        double heading = (-robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        double frontLeftPower = maxVelocity * ((rotY + rotX + rx) / denominator);
        double backLeftPower = maxVelocity * ((rotY - rotX + rx) / denominator);
        double frontRightPower = maxVelocity * ((rotY - rotX - rx) / denominator);
        double backRightPower = maxVelocity * ((rotY + rotX - rx) / denominator);

        // setting each power determined previously from the math above
        // as well as multiplying it by a drive power that can be changed.
        robot.bl.setVelocity(backLeftPower * drivePower + testPower);
        robot.br.setVelocity(backRightPower * drivePower + testPower);
        robot.fl.setVelocity(frontLeftPower * drivePower + testPower);
        robot.fr.setVelocity(frontRightPower * drivePower + testPower);
    }

    @Override
    public void init() {

    }

    @Override
    public void exec() {
        if (recordVelo < robot.bl.getVelocity()){
            recordVelo = robot.bl.getVelocity();
        }
        DriveTrainControl();
    }

    @Override
    public void telemetry() {

        engine.telemetry.addData("imu yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("imu pitch", robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        engine.telemetry.addData("imu roll", robot.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        engine.telemetry.addData("Max Velocity", recordVelo);

    }

    @Override
    public void buttonDown(Gamepad gamepad, String button) {
    }
}