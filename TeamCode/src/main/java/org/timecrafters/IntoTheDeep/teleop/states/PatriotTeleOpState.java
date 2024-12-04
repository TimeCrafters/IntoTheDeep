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
    public int differential;
    public boolean intakeManualControl = false;
    public boolean depositManualControl = false;
    public double intakeArmPower = 4;
    public double depositorArmPower = 2;
    public int intakeTarget = 0;
    public int depositTarget = 0;

    public int leftDifTarget = 0;
    public int rightDifTarget = 0;
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
        } else if (engine.gamepad2.a && engine.gamepad2.left_bumper && engine.gamepad2.right_bumper) {
            armPos = "Transfer 2/2";
        } else if (engine.gamepad2.y && !engine.gamepad2.left_bumper) {
            armPos = "Deposit Spec";
        } else if (engine.gamepad2.y && engine.gamepad2.left_bumper) {
            armPos = "Deposit Basket";
        } else if (engine.gamepad2.x) {
            armPos = "Intake";
        } else if (engine.gamepad2.b && !engine.gamepad2.start) {
            armPos = "Reset";
        }
    }


    public void ArmSequencer() {

        if (armPos.equals("Default")) {
            differential = 0;

            if (robot.intakeExtendo.getCurrentPosition() > 300 || robot.intakeExtendo.getCurrentPosition() < 800) {

                robot.intakeClaw.setPosition(robot.IntakeCLOSE);
                robot.depoClaw.setPosition(robot.DepoOPEN);
                robot.depoRight.setPosition(robot.depoDefaultPos);
                robot.depoLeft.setPosition(robot.depoDefaultPos);

            }

        }
        if (armPos.equals("Transfer 1/2")) {
            differential = 0;
            robot.intakeClaw.setPosition(robot.IntakeCLOSE);
            robot.depoClaw.setPosition(robot.DepoCLOSE);
            robot.depoRight.setPosition(robot.depoDefaultPos);
            robot.depoLeft.setPosition(robot.depoDefaultPos);

        }
        if (armPos.equals("Transfer 2/2")) {
            differential = 0;
            robot.intakeClaw.setPosition(robot.IntakeOPEN);
            robot.depoClaw.setPosition(robot.DepoCLOSE);
            robot.depoRight.setPosition(robot.depoDefaultPos);
            robot.depoLeft.setPosition(robot.depoDefaultPos);

        }
        if (armPos.equals("Deposit Spec")) {
            differential = 0;
            robot.intakeClaw.setPosition(robot.IntakeOPEN);
            robot.depoRight.setPosition(robot.depoSpecPos);
            robot.depoLeft.setPosition(robot.depoSpecPos);

        }
        if (armPos.equals("Deposit Basket")) {
            differential = 0;
            robot.intakeClaw.setPosition(robot.IntakeOPEN);
            robot.depoRight.setPosition(robot.depoBasketPos);
            robot.depoLeft.setPosition(robot.depoBasketPos);

        }
        if (armPos.equals("Intake")) {

            robot.depoClaw.setPosition(robot.DepoOPEN);
            robot.depoRight.setPosition(robot.depoDefaultPos);
            robot.depoLeft.setPosition(robot.depoDefaultPos);

            if (engine.gamepad2.right_stick_x > 0) {
                differential -= 200;
            } else if (engine.gamepad2.right_stick_x < 0) {
                differential += 200;
            }
        }

        if (armPos.equals("Reset")) {
            robot.intakeClaw.setPosition(robot.IntakeCLOSE);
            robot.depoClaw.setPosition(robot.DepoOPEN);
            robot.rightDiff.setPower(engine.gamepad2.right_stick_y);
            robot.leftDiff.setPower(engine.gamepad2.left_stick_y);

        }
    }

    public void SlideAutomationControl(){

//        if (intakeManualControl){
//            robot.intakeExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.intakeExtendo.setPower(-engine.gamepad2.left_stick_y);
//        }
        if (depositManualControl){
            robot.depositLeftExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.depositRightExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.depositLeftExtendo.setPower(engine.gamepad2.left_stick_y);
            robot.depositRightExtendo.setPower(engine.gamepad2.left_stick_y);
        }
        if (!depositManualControl){
            robot.depositLeftExtendo.setTargetPosition(depositTarget);
            robot.depositRightExtendo.setTargetPosition(depositTarget);

            robot.depositLeftExtendo.setPower(depositorArmPower);
            robot.depositRightExtendo.setPower(depositorArmPower);

            robot.depositLeftExtendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.depositRightExtendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//        if (!intakeManualControl) {
//            robot.intakeExtendo.setTargetPosition(intakeTarget);
//            robot.intakeExtendo.setPower(intakeArmPower);
//            robot.intakeExtendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }


        if (armPos.equals("Default")){
            depositManualControl = false;
            depositTarget = -10;

            if (robot.intakeExtendo.getCurrentPosition() < 1300){
                leftDifTarget = 0;
                rightDifTarget = 0;
            } else {
                leftDifTarget = robot.intakeExposed + differential;
                rightDifTarget = robot.intakeExposed - differential;
            }

            if ((robot.posLeftDiffy <= 150 && robot.posLeftDiffy >= -150) && (robot.posRightDiffy <= 150 && robot.posRightDiffy >= -150)){
                intakeTarget = 0;
            } else {
                intakeTarget = 200;
            }



        }
        if (armPos.equals("Transfer 1/2")){
            depositManualControl = false;
            intakeTarget = -20;
            depositTarget = -10;

            leftDifTarget = 0;
            rightDifTarget = 0;
        }
        if (armPos.equals("Transfer 2/2")){
            depositManualControl = false;
            intakeTarget = 400;
            depositTarget = -10;

            leftDifTarget = 0;
            rightDifTarget = 0;

        }
        if (armPos.equals("Deposit Spec")){
            depositManualControl = true;
            intakeTarget = 400;

            leftDifTarget = 0;
            rightDifTarget = 0;
        }
        if (armPos.equals("Deposit Basket")){
            depositManualControl = true;
            intakeTarget = 400;
        }

        if (armPos.equals("Intake")){

            intakeTarget = 1550;

            if (robot.intakeExtendo.getCurrentPosition() >= 100){
                leftDifTarget = robot.intakeExposed + differential;
                rightDifTarget = robot.intakeExposed - differential;
            }

            depositManualControl = false;
            depositTarget = -10;


        }
    }

    @Override
    public void init() {
        robot.octoquad.resetAllPositions();
        robot.intakeExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void exec() {
        if (robot.intakeExtendo.getVelocity() > maxExtendoVelo){
            maxExtendoVelo = robot.intakeExtendo.getVelocity();
        }


        robot.OdoLocalizer();

        robot.readOctoQuad();

        // -------------------------------------------------------------------------------------------------------------- Player 1 (Wheels)
        DriveTrainControl();

        // ---------------------------------------------------------------------------------------------------------------- Player 2 (Toys)

        robot.extendoTarget = intakeTarget;
        robot.HorizontalExtendoControl();

        SlideAutomationControl();

        ArmPos();

        robot.leftDiff.setPower(robot.BasicPController(leftDifTarget, robot.posLeftDiffy, 0.0005, 1));
        robot.rightDiff.setPower(robot.BasicPController(rightDifTarget, robot.posRightDiffy, 0.0005, 1));

        ArmSequencer();





    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("imu yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("imu pitch", -robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        engine.telemetry.addData("imu roll", -robot.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        engine.telemetry.addData("posX", robot.posX);
        engine.telemetry.addData("posY", robot.posY);
        engine.telemetry.addData("posH", robot.posH);
        engine.telemetry.addData("horizontal extendo max Velo", maxExtendoVelo);
        engine.telemetry.addData("differential", differential);
        engine.telemetry.addData("left diffy pos", robot.posLeftDiffy);
        engine.telemetry.addData("right diffy pos", robot.posRightDiffy);
        engine.telemetry.addData("left difTarget", leftDifTarget);
        engine.telemetry.addData("right difTarget", rightDifTarget);
        engine.telemetry.addData("armPos", armPos);
        engine.telemetry.addData("max velo", maxVelocity);
        engine.telemetry.addData("intake extendo pos", robot.intakeExtendo.getCurrentPosition());
        engine.telemetry.addData("right diffy power", robot.BasicPController(leftDifTarget, robot.posRightDiffy, 0.0005, 1));
        engine.telemetry.addData("left diffy power", robot.BasicPController(rightDifTarget, robot.posLeftDiffy, 0.0005, 1));
        engine.telemetry.addData("depositManualControl", depositManualControl);
        engine.telemetry.addData("depo left servo pos", robot.depoLeft.getPosition());
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