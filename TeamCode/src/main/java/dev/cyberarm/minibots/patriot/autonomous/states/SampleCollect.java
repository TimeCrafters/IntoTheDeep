package dev.cyberarm.minibots.patriot.autonomous.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class SampleCollect extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;
    public SampleCollect(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        // STEPS:
        // Ensure depositor and intake claws are not handing off
        // Invert handoff, just in case, if applicable
        //
        // Extend extension AND stow lift
        // Position and open intake claw for collecting
        // Position and open depositor claw for handoff
    }

    @Override
    public void exec() {

    }
}
