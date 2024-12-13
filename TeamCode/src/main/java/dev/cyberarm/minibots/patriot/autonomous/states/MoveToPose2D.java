package dev.cyberarm.minibots.patriot.autonomous.states;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;

public class MoveToPose2D extends CyberarmState {
    private final MinibotPatriotRobot robot;
    private final String groupName, actionName;

    private final int timeoutMS;
    private final double targetXInches, targetYInches, targetHeadingDegrees, positionToleranceInches, headingToleranceDegrees;
    public MoveToPose2D(MinibotPatriotRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeout_ms").value();
        this.targetXInches = robot.config.variable(groupName, actionName, "target_x_inches").value();
        this.targetYInches = robot.config.variable(groupName, actionName, "target_y_inches").value();
        this.targetHeadingDegrees = robot.config.variable(groupName, actionName, "target_heading_degrees").value();
        this.positionToleranceInches = robot.config.variable(groupName, actionName, "position_tolerance_inches").value();
        this.headingToleranceDegrees = robot.config.variable(groupName, actionName, "heading_tolerance_degrees").value();
    }

    @Override
    public void start() {
        robot.setTargetPosition(new SparkFunOTOS.Pose2D(targetXInches, targetYInches, targetHeadingDegrees));
    }

    @Override
    public void exec() {
        if (runTime() >= timeoutMS) {
            finished();
            return;
        }

        if (Utilities.isBetween(robot.getPosition().x, robot.getTargetPosition().x - positionToleranceInches, robot.getTargetPosition().y + positionToleranceInches) &&
                Utilities.isBetween(robot.getPosition().y, robot.getTargetPosition().y - positionToleranceInches, robot.getTargetPosition().y + positionToleranceInches) &&
                Math.abs(Utilities.angleDiff(Utilities.facing(robot.getPosition().h), Utilities.facing(robot.getTargetPosition().h))) <= headingToleranceDegrees) {
            finished();
        }
    }
}
