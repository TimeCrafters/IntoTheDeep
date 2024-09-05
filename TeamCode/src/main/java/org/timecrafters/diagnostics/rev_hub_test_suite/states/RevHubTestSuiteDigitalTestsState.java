package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;

public class RevHubTestSuiteDigitalTestsState extends RevTestSuiteTestState {
    private ArrayList<DigitalChannel> sensors;
    private int sensor_index = 0;
    public RevHubTestSuiteDigitalTestsState(RevHubTestSuiteRobot robot) {
        super(robot);
    }

    @Override
    public void start() {
        super.start();

        sensors = robot.testingControlHub ? robot.controlHubDigitalSensors : robot.expansionHubDigitalSensors;
        robot.stage = STAGE.DIGITAL_SENSOR;
    }

    @Override
    public void exec() {
        super.exec();

        if (testComplete)
            return;

        if (sensor_index >= sensors.size()) {
            nextStage();
            report("-"); // Add newline
        }

        if (robot.stage.ordinal() > STAGE.DIGITAL_SENSOR.ordinal()) {
            testComplete = true;
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("DIGITAL SENSOR PORT TESTING");
        engine.telemetry.addLine();

        if (sensor_index < sensors.size()) {
            DigitalChannel sensor = sensors.get(sensor_index);

            engine.telemetry.addData("Digital Sensor", "Port: %d, State: %s", sensor_index, (sensor.getState() ? "ON" : "OFF"));
            engine.telemetry.addLine();

            engine.telemetry.addLine("MANUAL TEST");
            engine.telemetry.addLine("PRESS `A` if Digital Sensor " + sensor_index + " value is OKAY.");
            engine.telemetry.addLine();
            engine.telemetry.addLine("PRESS `Y` if Digital Sensor " + sensor_index + " value is NOT OKAY.");
            engine.telemetry.addLine();
        }


        super.telemetry();
    }

    @Override
    public void buttonUp(Gamepad gamepad, String button) {
        if (robot.stage != STAGE.DIGITAL_SENSOR)
            return;

        if (gamepad == engine.gamepad1) {
            if (button.equals("a")) {
                report("PASSED: Digital Sensor " + sensor_index + " OKAY");

                sensor_index++;
            } else if (button.equals("y")) {
                report("FAILED: Digital Sensor " + sensor_index + " NOT OKAY");

                sensor_index++;
            }
        }
    }
}
