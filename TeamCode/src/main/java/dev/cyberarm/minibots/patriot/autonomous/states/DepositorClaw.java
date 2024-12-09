package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot.HardwareState;

public class DepositorClaw extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;

    private final int timeoutMS;
    private final boolean openClaw;
    public DepositorClaw(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeout_ms").value();
        this.openClaw = robot.config.variable(groupName, actionName, "open_claw").value();
    }

    @Override
    public void start() {
        robot.positionClaw(openClaw ? HardwareState.DEPOSITOR_CLAW_OPEN : HardwareState.DEPOSITOR_CLAW_CLOSED);
    }

    @Override
    public void exec() {
        if (runTime() >= timeoutMS) {
            finished();
        }
    }
}
