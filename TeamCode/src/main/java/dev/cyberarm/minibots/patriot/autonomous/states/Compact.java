package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class Compact extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;
    public Compact(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        // STEPS:
        // Close intake and depositor claws
        // Ensure there is room for the depositor and intake claws by ensuring that the extension is out a bit
        //
        // Park intake claw AND Park depositor claw
        // Stow lift AND mostly stow extension
        // stow extension
    }

    @Override
    public void exec() {
    }
}
