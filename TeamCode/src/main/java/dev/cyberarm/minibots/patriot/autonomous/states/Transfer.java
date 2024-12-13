package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class Transfer extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;
    public Transfer(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        // STEPS:
        // Ensure that there is room for depositor and intake claws to co-exist
        // Stow lift AND position depositor claw
        // Stow intake claw and pre-position extension
        // Stow extension
        // PREVENT ABORT
        // Close depositor claw THEN open intake claw
        // ALLOW ABORT
        // Move extension out a bit
        //
        // CONTINUE
    }

    @Override
    public void exec() {

    }
}
