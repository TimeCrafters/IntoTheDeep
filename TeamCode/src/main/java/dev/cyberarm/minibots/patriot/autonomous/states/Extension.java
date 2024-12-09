package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class Extension extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;

    private final int timeoutMS;
    private final double targetReachInches, toleranceInches;
    public Extension(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeout_ms").value();
        this.targetReachInches = robot.config.variable(groupName, actionName, "target_reach_inches").value();
        this.toleranceInches = robot.config.variable(groupName, actionName, "tolerance_inches").value();
    }

    @Override
    public void start() {
        robot.setExtensionReach(targetReachInches, toleranceInches);
    }

    @Override
    public void exec() {
        if (runTime() >= timeoutMS) {
            // FIXME: Halt lift when timing out?
            finished();
            return;
        }

        if (Utilities.atTargetPosition(robot.extension)) {
            finished();
        }
    }
}
