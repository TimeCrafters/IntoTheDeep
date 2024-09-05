package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;
import java.util.Locale;

public class RevHubTestSuiteMotorTestsState extends RevTestSuiteTestState {
    private int motor_index = 0;

    private ArrayList<DcMotorEx> motors;
    private double lastMonitorTime, lastSampleMonitorTime;
    private final double automaticInterval = 3_000; // milliseconds
    private final double automaticSampleInterval = 3_000.0; // milliseconds - test motor current and ticks per second

    private final ArrayList<Double> sampleAmpsList = new ArrayList<>();
    private final ArrayList<Integer> sampleTicksList = new ArrayList<>();
    private int initialValue, lastValue;
    private boolean setInitialValue = true;
    public RevHubTestSuiteMotorTestsState(RevHubTestSuiteRobot robot) {
        super(robot);
    }

    @Override
    public void start() {
        super.start();

        motors = robot.testingControlHub ? robot.controlHubMotors : robot.expansionHubMotors;
        lastMonitorTime = runTime();

        robot.stage = STAGE.MOTOR_ENCODER_STEADY;
    }

    @Override
    public void exec() {
        super.exec();

        if (testComplete)
            return;

        switch (robot.stage) {
            case MOTOR_ENCODER_STEADY: {
                test_encoder_steady();
                break;
            }
            case MOTOR_ENCODER_FORWARD: {
                test_encoder_changes(true);
                break;
            }
            case MOTOR_ENCODER_REVERSE: {
                test_encoder_changes(false);
                break;
            }
            case MOTOR_50_PERCENT_SPEED: {
                test_motor_current_and_ticks_per_second(0.5);
                break;
            }
            case MOTOR_100_PERCENT_SPEED: {
                test_motor_current_and_ticks_per_second(1.0);
                break;
            }
            case MOTOR_BRAKING_MODE: {
                test_motor_braking();
                break;
            }
        }

        if (robot.stage.ordinal() > STAGE.MOTOR_BRAKING_MODE.ordinal()) {
            testComplete = true;
        }
    }

    protected void test_encoder_steady() {
        if (motor_index >= motors.size()) {
            nextStage();
            motor_index = 0;
            setInitialValue = true;

            report("-"); // Embed newline

            return;
        }

        DcMotorEx motor = motors.get(motor_index);

        if (setInitialValue) {
            setInitialValue = false;
            initialValue = motor.getCurrentPosition();
            lastMonitorTime = runTime();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (runTime() - lastMonitorTime >= automaticInterval) {
            lastValue = Math.abs(initialValue - motor.getCurrentPosition());

            if (lastValue <= 2) {
                report("PASSED: Motor Encoder " + motor_index + " STEADY");
            } else {
                report("FAILED: Motor Encoder " + motor_index + " STEADY (" + lastValue + " > 2)");
            }

            motor_index++;
            setInitialValue = true;
        }
    }

    protected void test_encoder_changes(boolean forward) {
        if (motor_index >= motors.size()) {
            nextStage();
            motor_index = 0;
            setInitialValue = true;

            report("-"); // Embed newline

            return;
        }

        DcMotorEx motor = motors.get(motor_index);

        if (setInitialValue) {
            setInitialValue = false;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            initialValue = motor.getCurrentPosition();
            lastMonitorTime = runTime();

            motor.setPower(forward ? 0.5 : -0.5);
        }

        if (runTime() - lastMonitorTime >= automaticInterval) {
            lastValue = initialValue - motor.getCurrentPosition();

            String pass = forward ? "CHANGED; FORWARD." : "CHANGED; REVERSE.";
            String fail = forward ? "NO CHANGE; FORWARD." : "NO CHANGE; REVERSE.";

            if (Math.abs(lastValue) >= 100) {
                report("PASSED: Motor Encoder `" + motor_index + "` " + pass);
            } else {
                report("FAILED: Motor Encoder `" + motor_index + "` " + fail + " (" + lastValue + " < " + (forward ? "" : "-") + "100)");
            }

            motor.setPower(0.0);
            motor_index++;
            setInitialValue = true;
        }
    }

    protected void test_motor_current_and_ticks_per_second(double speed) {
        if (motor_index >= motors.size()) {
            nextStage();
            motor_index = 0;
            setInitialValue = true;

            report("-"); // Embed newline

            return;
        }

        DcMotorEx motor = motors.get(motor_index);

        if (setInitialValue) {
            setInitialValue = false;

            // Reset samples
            sampleAmpsList.clear();
            sampleTicksList.clear();

            initialValue = 0;
            lastMonitorTime = runTime();
            lastSampleMonitorTime = runTime();

            // Start "warming up" motor
            motor.setPower(speed);
        }


        if (runTime() - lastMonitorTime >= automaticSampleInterval) {
            if (initialValue == 0) {
                initialValue = 1;
                lastMonitorTime = runTime();

                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (initialValue == 1) {
            // Collect stats
            if (runTime() - lastSampleMonitorTime >= 100) {
                lastSampleMonitorTime = runTime();

                sampleAmpsList.add(motor.getCurrent(CurrentUnit.AMPS));
                sampleTicksList.add(motor.getCurrentPosition());
            }

            /* --- Generate report --- */
            if (runTime() - lastMonitorTime >= automaticSampleInterval) {
                double average_amps = 0.0;
                double average_ticks = 0.0; // NOTE: needs to be converted into PER SECOND "space"
                double total_amps = 0.0, total_ticks = 0.0;

                for (double amps : sampleAmpsList) {
                    total_amps += amps;
                }

                for (double ticks : sampleTicksList) {
                    total_ticks += ticks;
                }

                average_amps = total_amps / sampleAmpsList.size();
                average_ticks = total_ticks / sampleTicksList.size();
                average_ticks = average_ticks * (1.0 / (automaticSampleInterval * 0.001)); // Convert to PER SECOND

                report(String.format(Locale.US, "RESULT: Motor %d | %.2f: AMPS: %.4f, TICKS/s: %d", motor_index, motor.getPower(), average_amps, (int)average_ticks));

                motor_index++;
                motor.setPower(0.0);
                setInitialValue = true;
            }
        }
    }

    protected void test_motor_braking() {
        if (motor_index >= motors.size()) {
            nextStage();
            motor_index = 0;
            setInitialValue = true;

            report("-"); // Embed newline

            return;
        }

        DcMotorEx motor = motors.get(motor_index);

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void buttonUp(Gamepad gamepad, String button) {
        if (robot.stage != STAGE.MOTOR_BRAKING_MODE) {
            return;
        }

        if (engine.gamepad1 == gamepad) {
            if (button.equals("a")) {
                report("PASSED: Motor " + motor_index + " BRAKES");

                try {
                    DcMotorEx motor = motors.get(motor_index);
                    motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                } catch (IndexOutOfBoundsException e) { /* no op */ }

                motor_index++;
                setInitialValue = true;
            } else if (button.equals("y")) {
                report("FAILED: Motor " + motor_index + " does NOT BRAKE");

                try {
                    DcMotorEx motor = motors.get(motor_index);
                    motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                } catch (IndexOutOfBoundsException e) { /* no op */ }

                motor_index++;
                setInitialValue = true;
            }
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("MOTOR CONTROLLER TESTING");
        engine.telemetry.addLine();

        if (motor_index < 4) {
            DcMotorEx motor = motors.get(motor_index);

            engine.telemetry.addData("Motor", "Port: %d: Power: %.2f, Velocity: %.2f, Ticks: %d", motor_index, motor.getPower(), motor.getVelocity(), motor.getCurrentPosition());
            engine.telemetry.addLine();
        }

        if (robot.stage == STAGE.MOTOR_BRAKING_MODE) {
            engine.telemetry.addLine("MANUAL TEST");
            engine.telemetry.addLine("PRESS `A` if Motor " + motor_index + " is BRAKING.");
            engine.telemetry.addLine();
            engine.telemetry.addLine("PRESS `Y` if Motor " + motor_index + " is NOT BRAKING.");
            engine.telemetry.addLine();
        }

        super.telemetry();
    }
}
