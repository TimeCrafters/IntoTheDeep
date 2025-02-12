package org.timecrafters.IntoTheDeep.teleop.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.IntoTheDeep.common.PatriotMk2;


import dev.cyberarm.engine.V2.CyberarmState;
public class PatriotMk2TeleOpState extends CyberarmState {

    private final PatriotMk2 robot;

    private double drivePower = 1;
    private double maxVelocity = 2680 * 0.90;

    private boolean lbsVar;

    private double testPower = 0;
    private double rx;
    private boolean headingLock;
    private double headingTarget;

    public PatriotMk2TeleOpState(PatriotMk2 robot) {
        this.robot = robot;
    }

    public void DriveTrainControl() {

        // -------------------------------------------------------------------------------------------------------------- Player 1 (Wheels)

        if (engine.gamepad1.right_stick_x != 0){
            headingLock = false;
        } if (engine.gamepad1.dpad_down) {
            headingLock = true;
            headingTarget = Math.toRadians(180);
        } else if (engine.gamepad1.dpad_up) {
            headingLock = true;
            headingTarget = Math.toRadians(0);
        } else if (engine.gamepad1.dpad_left) {
            headingLock = true;
            headingTarget = Math.toRadians(90);
        } else if (engine.gamepad1.dpad_right || engine.gamepad1.x) {
            headingLock = true;
            headingTarget = Math.toRadians(-90);
        } else if (engine.gamepad1.y){
            headingTarget = Math.toRadians(-45);
        }

        if (headingLock){
            rx = robot.HeadingPIDControl(headingTarget, robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        } else {
            rx = engine.gamepad1.right_stick_x;
        }

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

    public void ArmModes(){

        if (engine.gamepad1.y && engine.gamepad1.left_trigger != 0){
            robot.armMode = "basket-low";
        }

        if (engine.gamepad1.y && engine.gamepad1.left_trigger == 0 && robot.armMode.equals("stowed")){
            robot.armMode = "basket-high";
        }

        if (engine.gamepad1.x && robot.armMode.equals("stowed")){
            robot.armMode = "intake-far";
        }

        if (engine.gamepad1.b){
            robot.armMode = "intake-close";
        }

        if (engine.gamepad1.a){
            robot.armMode = "stowed";
        }


    }

    public void ServoModes(){

        robot.twist.setPosition(robot.twistPos);
        robot.claw.setPosition(robot.clawPos);
        robot.rightPivot.setPosition(robot.pivotPos);
        robot.leftPivot.setPosition(robot.pivotPos);

        if (robot.armMode.equals("basket-low") && engine.gamepad1.right_trigger == 0 || robot.armMode.equals("basket-high") && engine.gamepad1.right_trigger == 0){
            robot.servoMode = "basket";
        }
        if (robot.armMode.equals("basket-low") && engine.gamepad1.right_trigger != 0 || robot.armMode.equals("basket-high") && engine.gamepad1.right_trigger != 0){
            robot.servoMode = "basket-out";
        }


    }



    @Override
    public void init() {
        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.extensionLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.extensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void exec() {
        robot.TeleOrAuto = "tele";

        DriveTrainControl();
        ArmModes();
        ServoModes();
        robot.ArmModesPreset();

    }

    @Override
    public void telemetry() {

        engine.telemetry.addData("imu yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("imu pitch", robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        engine.telemetry.addData("imu roll", robot.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));

        engine.telemetry.addData("Heading lock enabled", headingLock);
        engine.telemetry.addData("Heading target", headingTarget);

        engine.telemetry.addData("Pivot Pos", robot.pivot.getCurrentPosition());
        engine.telemetry.addData("extension Pos", robot.extensionLeft.getCurrentPosition());

    }

    @Override
    public void buttonDown(Gamepad gamepad, String button) {
    }
}