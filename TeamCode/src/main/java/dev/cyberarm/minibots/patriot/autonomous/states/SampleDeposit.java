package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class SampleDeposit extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;
    public SampleDeposit(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        // STEPS:
        // TODO
    }

    @Override
    public void exec() {

    }
}