package org.timecrafters.IntoTheDeep.teleop.states;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.IntoTheDeep.common.Patriot;
import com.acmerobotics.dashboard.config.Config;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class PatriotTeleOpState extends CyberarmState {

    private final Patriot robot;
    private double drivePower = 1;
    private double maxVelocity = 2320 * 0.8;
    private boolean lbsVar;
    private String armPos = "default";
    public static double depoPivotPos = 0;
    public double IntakeCLOSE = 1;
    public double DepoCLOSE = 1;
    public double IntakeOPEN = 0.4;
    public double DepoOPEN = 0.7;
    public double intakeOrigin = 0;
    public double intakeExposed = 5100;
    public double tolerance = 300;
    public long lastExecutedTime;
    public int differential;


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

    public void ArmPos(){
        if (engine.gamepad2.a && !engine.gamepad2.left_bumper && !engine.gamepad2.right_bumper) {
            armPos = "Default";
        } else if (engine.gamepad2.a && engine.gamepad2.left_bumper && !engine.gamepad2.right_bumper){
            armPos = "Transfer 1/2";
        } else if (engine.gamepad2.a && engine.gamepad2.left_bumper && engine.gamepad2.right_bumper){
            armPos = "Transfer 2/2";
        } else if (engine.gamepad2.y && !engine.gamepad2.left_bumper){
            armPos = "Deposit 1/2";
        } else if (engine.gamepad2.y && engine.gamepad2.left_bumper) {
            armPos = "Deposit 2/2";
        } else if (engine.gamepad2.x){
            armPos = "Intake";
        } else if (engine.gamepad2.b){
            armPos = "Reset";
        }
    }

    public void ArmSequencer() {
        if (armPos.equals("Default")) {
            differential = 0;
            robot.intakeClaw.setPosition(IntakeCLOSE);
            robot.depoClaw.setPosition(DepoOPEN);
//            robot.depoRight.setPosition(0);
            robot.depoLeft.setPosition(0);
            if (robot.posLeftDiffy > intakeOrigin + tolerance) {
                robot.leftDiff.setPower(0.5);
            } else if (robot.posLeftDiffy < intakeOrigin - tolerance) {
                robot.leftDiff.setPower(-0.5);
            } else {
                robot.leftDiff.setPower(0);
            }
            if (robot.posRightDiffy > intakeOrigin + tolerance) {
                robot.rightDiff.setPower(0.5);
            } else if (robot.posRightDiffy < intakeOrigin - tolerance) {
                robot.rightDiff.setPower(-0.5);
            } else {
                robot.rightDiff.setPower(0);
            }

        }
        if (armPos.equals("Transfer 1/2")) {
            differential = 0;
            robot.intakeClaw.setPosition(IntakeCLOSE);
            robot.depoClaw.setPosition(DepoCLOSE);
//            robot.depoRight.setPosition(0);
            robot.depoLeft.setPosition(0);
            if (robot.posLeftDiffy > intakeOrigin + tolerance) {
                robot.leftDiff.setPower(0.5);
            } else if (robot.posLeftDiffy < intakeOrigin - tolerance) {
                robot.leftDiff.setPower(-0.5);
            } else {
                robot.leftDiff.setPower(0);
            }
            if (robot.posRightDiffy > intakeOrigin + tolerance) {
                robot.rightDiff.setPower(0.5);
            } else if (robot.posRightDiffy < intakeOrigin - tolerance) {
                robot.rightDiff.setPower(-0.5);
            } else {
                robot.rightDiff.setPower(0);
            }

        }
        if (armPos.equals("Transfer 2/2")) {
            differential = 0;
            robot.intakeClaw.setPosition(IntakeOPEN);
            robot.depoClaw.setPosition(DepoCLOSE);
//            robot.depoRight.setPosition(0);
            robot.depoLeft.setPosition(0);
            if (robot.posLeftDiffy > intakeOrigin + tolerance) {
                robot.leftDiff.setPower(0.5);
            } else if (robot.posLeftDiffy < intakeOrigin - tolerance) {
                robot.leftDiff.setPower(-0.5);
            } else {
                robot.leftDiff.setPower(0);
            }
            if (robot.posRightDiffy > intakeOrigin + tolerance) {
                robot.rightDiff.setPower(0.5);
            } else if (robot.posRightDiffy < intakeOrigin - tolerance) {
                robot.rightDiff.setPower(-0.5);
            } else {
                robot.rightDiff.setPower(0);
            }

        }
        if (armPos.equals("Deposit 1/2")) {
            differential = 0;
            robot.intakeClaw.setPosition(IntakeCLOSE);
            robot.depoClaw.setPosition(DepoCLOSE);
//            robot.depoRight.setPosition(1);
            robot.depoLeft.setPosition(1);

            if (robot.posLeftDiffy > intakeOrigin + tolerance) {
                robot.leftDiff.setPower(0.5);
            } else if (robot.posLeftDiffy < intakeOrigin - tolerance) {
                robot.leftDiff.setPower(-0.5);
            } else {
                robot.leftDiff.setPower(0);
            }
            if (robot.posRightDiffy > intakeOrigin + tolerance) {
                robot.rightDiff.setPower(0.5);
            } else if (robot.posRightDiffy < intakeOrigin - tolerance) {
                robot.rightDiff.setPower(-0.5);
            } else {
                robot.rightDiff.setPower(0);
            }

        }
        if (armPos.equals("Deposit 2/2")) {
            differential = 0;
            robot.intakeClaw.setPosition(IntakeOPEN);
            robot.depoClaw.setPosition(DepoOPEN);
//            robot.depoRight.setPosition(1);
            robot.depoLeft.setPosition(0);
            if (robot.posLeftDiffy > intakeOrigin + tolerance) {
                robot.leftDiff.setPower(0.5);
            } else if (robot.posLeftDiffy < intakeOrigin - tolerance) {
                robot.leftDiff.setPower(-0.5);
            } else {
                robot.leftDiff.setPower(0);
            }
            if (robot.posRightDiffy > intakeOrigin + tolerance) {
                robot.rightDiff.setPower(0.5);
            } else if (robot.posRightDiffy < intakeOrigin - tolerance) {
                robot.rightDiff.setPower(-0.5);
            } else {
                robot.rightDiff.setPower(0);
            }

        }
        if (armPos.equals("Intake")) {
            robot.intakeClaw.setPosition(IntakeOPEN);
            robot.depoClaw.setPosition(DepoOPEN);
//            robot.depoRight.setPosition(0);
            robot.depoLeft.setPosition(0);

            if (robot.posLeftDiffy > intakeExposed + tolerance - differential) {
                robot.leftDiff.setPower(0.5);
            } else if (robot.posLeftDiffy < intakeExposed - tolerance - differential) {
                robot.leftDiff.setPower(-0.5);
            } else {
                robot.leftDiff.setPower(0);
            }
            if (robot.posRightDiffy > intakeExposed + tolerance + differential) {
                robot.rightDiff.setPower(0.5);
            } else if (robot.posRightDiffy < intakeExposed - tolerance + differential) {
                robot.rightDiff.setPower(-0.5);

            } else {
                robot.rightDiff.setPower(0);
            }

            if (engine.gamepad2.right_stick_x > 0) {
                differential += 100;
            } else if (engine.gamepad2.right_stick_x < 0) {
                differential -= 100;
            }
        }

        if (armPos.equals("Reset")) {
            robot.intakeClaw.setPosition(IntakeCLOSE);
            robot.depoClaw.setPosition(DepoOPEN);
            robot.rightDiff.setPower(engine.gamepad2.right_stick_y);
            robot.leftDiff.setPower(engine.gamepad2.left_stick_y);

        }
    }


    @Override
    public void init() {
        robot.octoquad.resetAllPositions();

    }

    @Override
    public void exec() {
        robot.readOctoQuad();


        // -------------------------------------------------------------------------------------------------------------- Player 1 (Wheels)
        DriveTrainControl();

        // ---------------------------------------------------------------------------------------------------------------- Player 2 (Toys)
        robot.intakeExtendo.setPower(-engine.gamepad2.left_stick_y * 1);
        robot.depositLeftExtendo.setPower(engine.gamepad2.right_trigger * 1);
        robot.depositRightExtendo.setPower(engine.gamepad2.right_trigger * 1);

        ArmPos();
        ArmSequencer();

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("imu yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("imu pitch", -robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        engine.telemetry.addData("imu roll", -robot.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        engine.telemetry.addData("differential", differential);
        engine.telemetry.addData("left diffy pos", robot.posLeftDiffy);
        engine.telemetry.addData("right diffy pos", robot.posRightDiffy);
        engine.telemetry.addData("armPos", armPos);
        engine.telemetry.addData("max velo", maxVelocity);
        engine.telemetry.addData("depo left servo pos", robot.depoLeft.getPosition());
    }
}