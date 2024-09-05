package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;

public class RevHubTestSuiteServoTestsState extends RevTestSuiteTestState {
    private int servo_index = 0;
    private ArrayList<Servo> servos;
    private double lastMonitorTime;
    private final double automaticInterval = 1_500.0; // milliseconds
    private boolean setInitialValue = true;
    public RevHubTestSuiteServoTestsState(RevHubTestSuiteRobot robot) {
        super(robot);
    }

    @Override
    public void start() {
        super.start();

        servos = robot.testingControlHub ? robot.controlHubServos : robot.expansionHubServos;
        lastMonitorTime = runTime();

        robot.stage = STAGE.SERVO_SWEEP;
    }

    @Override
    public void exec() {
        super.exec();

        if (testComplete)
            return;

        test_servos();

        if (robot.stage.ordinal() > STAGE.SERVO_SWEEP.ordinal()) {
            testComplete = true;
        }
    }

    private void test_servos() {
        if (servo_index >= servos.size()) {
            nextStage();
            return;
        }

        Servo servo = servos.get(servo_index);

        if (setInitialValue) {
            setInitialValue = false;
            lastMonitorTime = runTime();

            servo.setPosition(0);
        }

        if (runTime() - lastMonitorTime >= automaticInterval) {
            lastMonitorTime = runTime();

            servo.setPosition(servo.getPosition() == 0 ? 1 : 0);
        }
    }

    public void buttonUp(Gamepad gamepad, String button) {
        if (robot.stage != STAGE.SERVO_SWEEP) {
            return;
        }

        if (engine.gamepad1 == gamepad) {
            if (button.equals("a")) {
                report("PASSED: Servo " + servo_index + " ROTATES");

                setInitialValue = true;
                servo_index++;
            } else if (button.equals("y")) {
                report("FAILED: Servo " + servo_index + " does NOT ROTATE");

                setInitialValue = true;
                servo_index++;
            }
        }
    }

    public void telemetry() {
        engine.telemetry.addLine("SERVO CONTROLLER TESTING");
        engine.telemetry.addLine();

        if (robot.stage == STAGE.SERVO_SWEEP) {
            engine.telemetry.addLine("MANUAL TEST");
            engine.telemetry.addLine("PRESS `A` if Servo " + servo_index + " is ROTATING.");
            engine.telemetry.addLine();
            engine.telemetry.addLine("PRESS `Y` if Servo " + servo_index + " is NOT ROTATING.");
            engine.telemetry.addLine();
        }

        super.telemetry();
    }
}
