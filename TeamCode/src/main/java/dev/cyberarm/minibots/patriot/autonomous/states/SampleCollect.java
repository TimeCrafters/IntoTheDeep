package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot.HardwareState;

public class SampleCollect extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;
    private enum SampleCollectState {
        POSITION_EXTENSION_AND_STOW_LIFT,
        POSITION_AND_OPEN_INTAKE_CLAW_FOR_COLLECTING,
        POSITION_AND_OPEN_DEPOSITOR_CLAW_FOR_HANDOFF,
        END
    }
    private SampleCollectState state = SampleCollectState.POSITION_EXTENSION_AND_STOW_LIFT;
    private double elapsedTime = 0;
    public SampleCollect(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        // STEPS:
        // Extend extension AND stow lift
        // Position and open intake claw for collecting
        // Position and open depositor claw for handoff
    }

    @Override
    public void exec() {
        switch (state) {
            case POSITION_EXTENSION_AND_STOW_LIFT: {
                robot.positionExtension(HardwareState.EXTENSION_OUT);
                if (elapsedTime() >= 500) {
                    robot.positionLift(HardwareState.LIFT_STOW);
                    state = SampleCollectState.POSITION_AND_OPEN_INTAKE_CLAW_FOR_COLLECTING;
                    elapsedTime = runTime();
                }
                break;
            }
            case POSITION_AND_OPEN_INTAKE_CLAW_FOR_COLLECTING: {
                robot.positionClaw(HardwareState.INTAKE_CLAW_OPEN);
                robot.positionIntakeDifferential(HardwareState.INTAKE_DIFFERENTIAL_COLLECT, 0);
                state = SampleCollectState.POSITION_AND_OPEN_DEPOSITOR_CLAW_FOR_HANDOFF;
                elapsedTime = runTime();
                break;
            }
            case POSITION_AND_OPEN_DEPOSITOR_CLAW_FOR_HANDOFF: {
                // Ensure that the depositor claw is closed before moving as it doesn't fit between
                // the slides when open and gets stuck.
                robot.positionClaw(HardwareState.DEPOSITOR_CLAW_CLOSED);
                robot.positionDepositorArm(HardwareState.DEPOSITOR_ARM_STOW);
                if (elapsedTime() >= 1_500) {
                    robot.positionClaw(HardwareState.DEPOSITOR_CLAW_OPEN);
                    elapsedTime = runTime();
                }
                break;
            }
            case END: {
                finished();
            }
        }
    }

    private double elapsedTime() {
        return runTime() - elapsedTime;
    }
}
