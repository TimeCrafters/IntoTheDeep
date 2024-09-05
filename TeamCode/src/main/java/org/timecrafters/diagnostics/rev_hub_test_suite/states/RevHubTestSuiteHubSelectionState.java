package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class RevHubTestSuiteHubSelectionState extends CyberarmState {
    private RevHubTestSuiteRobot robot;
    public RevHubTestSuiteHubSelectionState(RevHubTestSuiteRobot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("SELECT HUB TO BE TESTED:");
        engine.telemetry.addLine();
        engine.telemetry.addLine("PRESS 'LEFT BUMPER' TO TEST CONTROL HUB");
        engine.telemetry.addLine("PRESS 'RIGHT BUMPER' TO TEST EXPANSION HUB");
        engine.telemetry.addLine();

        super.telemetry();
    }

    @Override
    public void buttonUp(Gamepad gamepad, String button) {
        if (button.equals("left_bumper")) {
            robot.testingControlHub = true;
            setHasFinished(true);
        }

        if (button.equals("right_bumper")) {
            robot.testingControlHub = false;
            setHasFinished(true);
        }
    }
}
