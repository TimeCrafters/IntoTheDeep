package org.timecrafters.IntoTheDeep.autonomous.states;

import org.timecrafters.IntoTheDeep.common.Patriot;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;

public class ClawDiffState extends CyberarmState {
    private String groupName, actionName;
    private Patriot robot;
    private int rightTargetPosition,leftTargetPosition;
    private int timeoutMS, tolerance;

    public ClawDiffState(Patriot robot, String groupName, String actionName) {
        this.groupName = groupName;
        this.actionName = actionName;

        this.robot = robot;
        this.rightTargetPosition = robot.configuration.variable(groupName, actionName, "rightTargetPosition").value();
        this.leftTargetPosition = robot.configuration.variable(groupName, actionName, "leftTargetPosition").value();
        this.timeoutMS = robot.configuration.variable(groupName, actionName, "timeoutMS").value();
        this.tolerance = robot.configuration.variable(groupName, actionName, "tolerance").value();
    }
    @Override
    public void exec() {
        if (runTime() >= timeoutMS) {
            robot.leftDiff.setPower(0);
            robot.rightDiff.setPower(0);

            finished();
            return;
        }

        if (Utilities.isBetween(robot.posLeftDiffy, leftTargetPosition-tolerance, leftTargetPosition+tolerance) &&
                Utilities.isBetween(robot.posRightDiffy, rightTargetPosition-tolerance, rightTargetPosition+tolerance))
        {
            robot.leftDiff.setPower(0);
            robot.rightDiff.setPower(0);

            finished();
        } else {
            robot.leftDiff.setPower(robot.BasicPController(leftTargetPosition, robot.posLeftDiffy, 0.0005, 1));
            robot.rightDiff.setPower(robot.BasicPController(rightTargetPosition, robot.posRightDiffy, 0.0005, 1));
        }

    }
}
