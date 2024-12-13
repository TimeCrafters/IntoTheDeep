package dev.cyberarm.minibots.patriot.autonomous.states;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class SetInitialPose2D extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;

    private final int timeoutMS;
    private final double xInches, yInches, hDegrees;
    public SetInitialPose2D(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeout_ms").value();
        this.xInches = robot.config.variable(groupName, actionName, "x_inches").value();
        this.yInches = robot.config.variable(groupName, actionName, "y_inches").value();
        this.hDegrees = robot.config.variable(groupName, actionName, "h_degrees").value();
    }

    @Override
    public void start() {
        robot.setInitialPosition(new SparkFunOTOS.Pose2D(xInches, yInches, hDegrees));
    }

    @Override
    public void exec() {
        finished();
    }
}
