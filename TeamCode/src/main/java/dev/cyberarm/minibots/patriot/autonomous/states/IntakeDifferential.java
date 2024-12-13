package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot.HardwareState;

public class IntakeDifferential extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;

    private final int timeoutMS, differential;
    private final boolean stowDifferential;
    public IntakeDifferential(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeout_ms").value();
        this.stowDifferential = robot.config.variable(groupName, actionName, "stowDifferential").value();
        this.differential = robot.config.variable(groupName, actionName, "differential").value();
    }

    @Override
    public void start() {
        robot.positionIntakeDifferential(
                stowDifferential ? HardwareState.INTAKE_DIFFERENTIAL_STOW : HardwareState.INTAKE_DIFFERENTIAL_COLLECT,
                differential);
    }

    @Override
    public void exec() {
        if (runTime() >= timeoutMS) {
            finished();
            return;
        }

        if (robot.intakeDifferentialAtPosition()) {
            finished();
        }
    }
}
