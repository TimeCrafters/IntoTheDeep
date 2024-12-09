package dev.cyberarm.minibots.patroit.teleop;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.patroit.common.MinibotPatriotRobot;

public class MinibotPatriotTeleOpEngine extends CyberarmEngine {
    @Override
    public void setup() {
        MinibotPatriotRobot robot = new MinibotPatriotRobot(this, false);

        addState(new MinibotPatriotTeleOpState(robot, "TeleOp", "00-00"));
    }
}
