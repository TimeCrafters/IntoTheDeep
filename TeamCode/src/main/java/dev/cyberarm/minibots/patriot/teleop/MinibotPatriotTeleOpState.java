package dev.cyberarm.minibots.patriot.teleop;

import static dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot.State;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class MinibotPatriotTeleOpState extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;
    public MinibotPatriotTeleOpState(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    @Override
    public void exec() {
        /* CONTROLS
            PANIC:
                START / OPTIONS: Panic, halt all motors and servos
            IMU:
                GUIDE: Reset
            DRIVETRAIN:
                LEFT JOYSTICK: X and Y movement
                RIGHT JOYSTICK X: Rotation
                LEFT JOYSTICK BTN / BACK / SHARE: Toggle Precision/Coarse speed
            EXTENSION:
            LIFT:
                LEFT and RIGHT TRIGGER: Manually raise/lower lift (Only controllable when robot state is: (SAMPLE_DEPOSIT, SPECIMEN_COLLECT, or SPECIMEN_DEPOSIT)
            INTAKE CLAW:
                LEFT and RIGHT TRIGGER: Rotate
                LEFT BUMPER: Toggle claw open/close
            DEPOSITOR CLAW:
                RIGHT BUMPER: Toggle claw open/close (only controllable when robot state is: SAMPLE_DEPOSIT, SPECIMEN_COLLECT, or SPECIMEN_DEPOSIT)

            A / (X): SAMPLE_COLLECT
            B / (O): SPECIMEN_COLLECT
            X / (□): SPECIMEN_DEPOSIT
            Y / (△): SAMPLE_DEPOSIT

            DPAD DOWN: COMPACT
            DPAD RIGHT: ACCENT LEVEL ONE
            DPAD LEFT: ACCENT LEVEL TWO
            DPAD UP ACCENT LEVEL THREE
         */

        if (robot.getState() == State.PANIC)
            return;

        robot.drivetrainRobotCentric(-engine.gamepad1.left_stick_y, engine.gamepad1.left_stick_x, engine.gamepad1.right_stick_x);
    }

    @Override
    public void telemetry() {
        robot.telemetry();
    }

    @Override
    public void buttonDown(Gamepad gamepad, String button) {
        if (gamepad == engine.gamepad1) {
            switch (button) {
                case "guide": {
                    robot.imu.resetYaw();
                    break;
                }
                case "left_joystick_button":
                case "back": {
                    robot.isPreciseDrivetrainVelocity = !robot.isPreciseDrivetrainVelocity;
                    break;
                }
                case "start": {
                    robot.togglePanic();
                    break;
                }
                case "a": {
                    robot.requestState(State.SAMPLE_COLLECT);
                    break;
                }
                case "b": {
                    robot.requestState(State.SPECIMEN_COLLECT);
                    break;
                }
                case "x": {
                    robot.requestState(State.SPECIMEN_DEPOSIT);
                    break;
                }
                case "y": {
                    robot.requestState(State.SAMPLE_DEPOSIT);
                    break;
                }
                case "dpad_down": {
                    robot.requestState(State.COMPACT);
                    break;
                }
                case "dpad_right": {
                    robot.requestState(State.ACCENT_LEVEL_ONE);
                    break;
                }
                case "dpad_left": {
                    robot.requestState(State.ACCENT_LEVEL_TWO);
                    break;
                }
                case "dpad_up": {
                    robot.requestState(State.ACCENT_LEVEL_THREE);
                    break;
                }
            }
        }
    }
}
