package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;

import dev.cyberarm.engine.V2.CyberarmState;

public class RevHubTestSuiteI2CTestsState extends RevTestSuiteTestState {
    private ArrayList<Rev2mDistanceSensor> sensors;
    private int sensor_index = 0;
    public RevHubTestSuiteI2CTestsState(RevHubTestSuiteRobot robot) {
        super(robot);
    }

    @Override
    public void start() {
        super.start();

        sensors = robot.testingControlHub ? robot.controlHubI2cSensors : robot.expansionHubI2cSensors;
        robot.stage = STAGE.I2C_SENSOR;
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

        if (robot.stage.ordinal() > STAGE.I2C_SENSOR.ordinal()) {
            testComplete = true;
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("I2C SENSOR PORT TESTING");
        engine.telemetry.addLine();

        if (sensor_index < 4) {
            Rev2mDistanceSensor sensor = sensors.get(sensor_index);

            engine.telemetry.addData("I2C Sensor", "Port: %d, Dist: %.2fMM", sensor_index, sensor.getDistance(DistanceUnit.MM));
            engine.telemetry.addLine();

            engine.telemetry.addLine("MANUAL TEST");
            engine.telemetry.addLine("PRESS `A` if I2C Sensor " + sensor_index + " value is OKAY.");
            engine.telemetry.addLine();
            engine.telemetry.addLine("PRESS `Y` if I2C Sensor " + sensor_index + " value is NOT OKAY.");
            engine.telemetry.addLine();
        }


        super.telemetry();
    }

    @Override
    public void buttonUp(Gamepad gamepad, String button) {
        if (gamepad == engine.gamepad1) {
            if (button.equals("a")) {
                report("PASSED: I2C Sensor " + sensor_index + " OKAY");

                sensor_index++;
            } else if (button.equals("y")) {
                report("FAILED: I2C Sensor " + sensor_index + " NOT OKAY");

                sensor_index++;
            }
        }
    }
}
