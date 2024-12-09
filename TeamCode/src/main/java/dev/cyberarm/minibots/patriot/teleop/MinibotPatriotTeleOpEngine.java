package dev.cyberarm.minibots.patriot.teleop;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class MinibotPatriotTeleOpEngine extends CyberarmEngine {
    @Override
    public void setup() {
        final MinibotPatriotRobot robot;
        if (MinibotPatriotRobot.getInstance() != null) {
            robot = MinibotPatriotRobot.getInstance();
        } else {
            robot = new MinibotPatriotRobot(this, false);
        }

        robot.setTeleOp();

        addState(new MinibotPatriotTeleOpState(robot, "TeleOp", "00-00"));
    }
}
