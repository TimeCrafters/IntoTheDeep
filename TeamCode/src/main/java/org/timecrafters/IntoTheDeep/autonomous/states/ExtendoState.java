package org.timecrafters.IntoTheDeep.autonomous.states;

import org.timecrafters.IntoTheDeep.common.Patriot;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;

public class ExtendoState extends CyberarmState {
    private String groupName, actionName;
    private Patriot robot;
    private int targetPosition;
    private int timeoutMS;

    public ExtendoState(Patriot robot, String groupName, String actionName) {
        this.groupName = groupName;
        this.actionName = actionName;

        this.robot = robot;
        this.targetPosition = robot.configuration.variable(groupName, actionName, "targetPosition").value();
        this.timeoutMS = robot.configuration.variable(groupName, actionName, "timeoutMS").value();
    }

    @Override
    public void start() {
        robot.extendoTarget = targetPosition;
    }

    @Override
    public void exec() {
        if (runTime() >= timeoutMS) {
            robot.intakeExtendo.setVelocity(0);

            finished();
            return;
        }

        robot.HorizontalExtendoControl();

        if (Utilities.isBetween(robot.intakeExtendo.getCurrentPosition(), robot.extendoTarget - 10, robot.extendoTarget + 10)) {
            robot.intakeExtendo.setVelocity(0);
            finished();
        }
    }
}
