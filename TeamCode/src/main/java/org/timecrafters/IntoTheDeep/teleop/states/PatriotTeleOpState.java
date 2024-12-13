package org.timecrafters.IntoTheDeep.teleop.states;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.IntoTheDeep.common.Patriot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.CyberarmState;

public class PatriotTeleOpState extends CyberarmState {

    private final Patriot robot;
    private double drivePower = 1;
    private String collectorToggle = "open";
    private double maxVelocity = 2320 * 0.8;
    private boolean lbsVar;
    private String armPos = "default";
    public boolean depositManualControl = false;
    public double depositorArmPower = 2;
//    public int intakeTarget = 0;
    public int depositTarget = 0;
//
//    public int leftDifTarget = 0;
//    public int rightDifTarget = 0;
    public double maxExtendoVelo = 0;




    public PatriotTeleOpState(Patriot robot) {
        this.robot = robot;
    }

    public void DriveTrainControl() {


        // -------------------------------------------------------------------------------------------------------------- Player 1 (Wheels)

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

        double frontLeftPower = maxVelocity * ((rotY + rotX + rx) / denominator);
        double backLeftPower = maxVelocity * ((rotY - rotX + rx) / denominator);
        double frontRightPower = maxVelocity * ((rotY - rotX - rx) / denominator);
        double backRightPower = maxVelocity * ((rotY + rotX - rx) / denominator);

        // setting each power determined previously from the math above
        // as well as multiplying it by a drive power that can be changed.
        robot.bl.setVelocity(backLeftPower * drivePower);
        robot.br.setVelocity(backRightPower * drivePower);
        robot.fl.setVelocity(frontLeftPower * drivePower);
        robot.fr.setVelocity(frontRightPower * drivePower);
    }

    public void ArmPos() {
        if (engine.gamepad2.a && !engine.gamepad2.left_bumper && !engine.gamepad2.right_bumper) {
            armPos = "Default";
        } else if (engine.gamepad2.a && engine.gamepad2.left_bumper && !engine.gamepad2.right_bumper) {
            armPos = "Transfer 1/2";
            robot.initialTime = System.currentTimeMillis();
        } else if (engine.gamepad2.a && engine.gamepad2.left_bumper && engine.gamepad2.right_bumper) {
            armPos = "Transfer 2/2";
            robot.initialTime = System.currentTimeMillis();
        } else if (engine.gamepad2.b) {
            armPos = "Deposit Spec Collecting";
            robot.depositFlip = false;
        } else if (engine.gamepad2.left_bumper && armPos.equals("Deposit Spec Collecting")) {
            armPos = "Deposit Spec Collected";
            robot.depositFlip = true;
        } else if (engine.gamepad2.y) {
            armPos = "Deposit Basket";
        } else if (engine.gamepad2.x) {
            armPos = "Intake";
        }
    }

    public double BasicPController(int target, int currentPos, double Kp, double maxPower) {
        int error = (target - currentPos);
        double power = error * Kp;

        if (power > maxPower) {
            power = maxPower;
        }
        if (power < -maxPower) {
            power = -maxPower;
        }
        return power;
    }

//

    @Override
    public void init() {
        robot.octoquad.resetAllPositions();
        robot.intakeExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void exec() {

        robot.autonomous = false;

        robot.readOctoQuad();
        ArmPos();
        robot.armPos = armPos;
        robot.ArmSequencer();
        robot.extendoTarget = robot.intakeTarget;
        robot.HorizontalExtendoControl();
        DriveTrainControl();

        if (engine.gamepad1.dpad_up){
            robot.winch.setPower(1);
        } else if (engine.gamepad1.dpad_down){
            robot.winch.setPower(-1);
        } else robot.winch.setPower(0);


        if (robot.depositManualControl){
            robot.depositLeftExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.depositRightExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.depositLeftExtendo.setPower(engine.gamepad2.left_stick_y);
            robot.depositRightExtendo.setPower(engine.gamepad2.left_stick_y);
        }
        if (!robot.depositManualControl){
            robot.DepoExtendoControl();
        }

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("depo slide Pos Left", robot.depositLeftExtendo.getCurrentPosition());
        engine.telemetry.addData("depo slide Pos Right", robot.depositRightExtendo.getCurrentPosition());
        engine.telemetry.addData("imu yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("imu pitch", -robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        engine.telemetry.addData("imu roll", -robot.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        engine.telemetry.addData("posX", robot.posX);
        engine.telemetry.addData("posY", robot.posY);
        engine.telemetry.addData("posH", robot.posH);
        engine.telemetry.addData("pidExtendo", robot.pidExtendo);
        engine.telemetry.addData("horizontal extendo max Velo", maxExtendoVelo);
        engine.telemetry.addData("differential", robot.differential);
        engine.telemetry.addData("left diffy pos", robot.posIntakeLeftDiffy);
        engine.telemetry.addData("right diffy pos", robot.posIntakeRightDiffy);
        engine.telemetry.addData("left diffy pos", robot.posDepositLeftDiffy);
        engine.telemetry.addData("right diffy pos", robot.posDepositRightDiffy);
        engine.telemetry.addData("left difTarget", robot.leftIntakeDifTarget);
        engine.telemetry.addData("right difTarget", robot.rightIntakeDifTarget);
        engine.telemetry.addData("armPos", armPos);
        engine.telemetry.addData("max velo", maxVelocity);
        engine.telemetry.addData("intake extendo pos", robot.intakeExtendo.getCurrentPosition());
        engine.telemetry.addData("right diffy power", BasicPController(robot.leftIntakeDifTarget, robot.posIntakeRightDiffy, 0.0005, 1));
        engine.telemetry.addData("left diffy power", BasicPController(robot.rightIntakeDifTarget, robot.posIntakeLeftDiffy, 0.0005, 1));
        engine.telemetry.addData("right diffy power", BasicPController(robot.leftIntakeDifTarget, robot.posIntakeRightDiffy, 0.0005, 1));
        engine.telemetry.addData("left diffy power", BasicPController(robot.rightIntakeDifTarget, robot.posIntakeLeftDiffy, 0.0005, 1));
        engine.telemetry.addData("depositManualControl", robot.depositManualControl);
        engine.telemetry.addData("intake extendo", robot.intakeExtendo.getCurrentPosition());
    }

    @Override
    public void buttonDown(Gamepad gamepad, String button) {
        if  (gamepad == engine.gamepad2) {
            if (button.equals("left_stick_button")) {
//                collectorOpen = !collectorOpen;
                // toggle servo position
                if (collectorToggle.equals("Open")) {
                    collectorToggle = "Close";
                } else {
                    collectorToggle = "Open";
                }

                if (collectorToggle.equals("Open")){
                    robot.intakeClaw.setPosition(robot.IntakeOPEN);
                    robot.depoClaw.setPosition(robot.DepoOPEN);
                } else {
                    robot.intakeClaw.setPosition(robot.IntakeCLOSE);
                    robot.depoClaw.setPosition(robot.DepoCLOSE);
                }
            }
        }
    }
}