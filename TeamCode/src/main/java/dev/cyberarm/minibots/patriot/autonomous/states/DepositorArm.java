package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot.HardwareState;

public class DepositorArm extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;

    private final int timeoutMS;
    private final boolean stowArm;
    public DepositorArm(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeout_ms").value();
        this.stowArm = robot.config.variable(groupName, actionName, "stow_arm").value();
    }

    @Override
    public void start() {
        robot.positionDepositorArm(stowArm ? HardwareState.DEPOSITOR_ARM_STOW : HardwareState.DEPOSITOR_ARM_DEPOSIT);
    }

    @Override
    public void exec() {
        if (runTime() >= timeoutMS) {
            finished();
        }
    }
}
