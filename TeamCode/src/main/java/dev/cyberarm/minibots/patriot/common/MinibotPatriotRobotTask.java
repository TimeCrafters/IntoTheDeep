package dev.cyberarm.minibots.patriot.common;

import dev.cyberarm.engine.V2.CyberarmState;

public class MinibotPatriotRobotTask extends CyberarmState {
    final MinibotPatriotRobot robot;

    public MinibotPatriotRobotTask(MinibotPatriotRobot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.update();
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("+++");
        robot.telemetry();
        engine.telemetry.addLine("+-+");
    }

    @Override
    public void stop() {
        robot.teardown();
    }
}
