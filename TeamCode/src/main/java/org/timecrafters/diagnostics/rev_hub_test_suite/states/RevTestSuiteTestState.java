package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import android.util.Log;

import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;

import dev.cyberarm.engine.V2.CyberarmState;

public class RevTestSuiteTestState extends CyberarmState {
    public enum STAGE {
        NONE,
        MOTOR_ENCODER_STEADY, /* Encoder value doesn't change by more than 2 ticks while motor is off. */
        MOTOR_ENCODER_FORWARD, /* Encoder value is increasing while motor is running */
        MOTOR_ENCODER_REVERSE, /* Encoder value is decreasing while motor is running */
        MOTOR_50_PERCENT_SPEED, /* Measure motor Amps and ticks per second at 50% speed, after a 3 second warm up */
        MOTOR_100_PERCENT_SPEED, /* Measure motor Amps and ticks per second at 50% speed, after a 3 second warm up */
        MOTOR_BRAKING_MODE, /* MANUAL: Test  */
        SERVO_SWEEP,
        ANALOG_SENSOR,
        DIGITAL_SENSOR,
        I2C_SENSOR,
        IMU,

        COMPLETE
    }

    final String TAG = "RevTestSuite|State";
    RevHubTestSuiteRobot robot;
    protected boolean testComplete = false;
    public RevTestSuiteTestState(RevHubTestSuiteRobot robot) {
        super();

        this.robot = robot;
    }

    @Override
    public void exec() {
        if (testComplete && engine.gamepad1.guide) {
            setHasFinished(true);
            testComplete = false;
        }
    }

    void report(String reason) {
        synchronized (robot.reports) {
            robot.reports.add(reason);
        }
    }

    public void nextStage() {
        if (robot.stage == STAGE.COMPLETE)
            return;

        robot.stage = STAGE.values()[robot.stage.ordinal() + 1];
    }

    @Override
    public void telemetry() {
        if (testComplete) {
            engine.telemetry.addLine("PRESS `GUIDE` TO CONTINUE");
        } else {
            engine.telemetry.addLine("TESTING");
        }

        engine.telemetry.addLine();
        engine.telemetry.addData("TEST MODE", (robot.testingControlHub ? "Control Hub" : "Expansion Hub"));
        engine.telemetry.addLine();

        engine.telemetry.addLine();
        engine.telemetry.addData("STAGE", robot.stage);
        engine.telemetry.addLine();
        engine.telemetry.addLine("REPORTS");

        synchronized (robot.reports) {
            for (String report : robot.reports) {
                engine.telemetry.addLine(report);
            }
        }
    }

    @Override
    public void setHasFinished(boolean value) {
        Log.i(TAG, "finished state: " + this.getClass() +", stage: " + robot.stage.toString());

        super.setHasFinished(value);
    }
}
