package dev.cyberarm.minibots.patriot.autonomous;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class MinibotPatriotAutonomousBasketSideEngine extends CyberarmEngine {
    @Override
    public void setup() {
        final MinibotPatriotRobot robot = new MinibotPatriotRobot(this, true);

        setupFromConfig(
                robot.config,
                "dev.cyberarm.minibots.patriot.autonomous",
                robot,
                MinibotPatriotRobot.class,
                "AutonomousBasketSide");
    }
}
