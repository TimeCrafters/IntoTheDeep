package org.timecrafters.IntoTheDeep.autonomous.states;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.IntoTheDeep.common.Patriot;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class DriveToCoordinatesState extends CyberarmState {

    Patriot robot;
    public double xTarget;
    public double yTarget;
    public double hTarget;
    public double maxXPower;
    public double maxYPower;
    public double maxHPower;
    private String actionName;

    public static int extendoTarget;

    public DriveToCoordinatesState(Patriot robot, String groupName, String actionName) {
        this.actionName = actionName;

        this.robot = robot;
        this.xTarget = robot.configuration.variable(groupName, actionName, "xTarget").value();
        this.yTarget = robot.configuration.variable(groupName, actionName, "yTarget").value();
        this.hTarget = robot.configuration.variable(groupName, actionName, "hTarget").value();
        this.maxXPower = robot.configuration.variable(groupName, actionName, "maxXPower").value();
        this.maxYPower = robot.configuration.variable(groupName, actionName, "maxYPower").value();
        this.maxHPower = robot.configuration.variable(groupName, actionName, "maxHPower").value();
    }

    @Override
    public void start() {
        super.start();
        robot.hTarget = hTarget;
        robot.yTarget = yTarget;
        robot.xTarget = xTarget;
        robot.yMaxPower = maxYPower;
        robot.xMaxPower = maxXPower;
        robot.hMaxPower = maxHPower;
        robot.extendoTarget = extendoTarget;
    }

    @Override
    public void exec() {

        if (Math.abs(robot.posX - robot.xTarget) < 2.5
                && Math.abs(robot.posY - robot.yTarget) < 2.5
                && Math.abs(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) - Math.abs(Math.toDegrees(robot.hTarget)) < 1) {
            setHasFinished(true);
        }
    }


    @Override
    public void telemetry() {
        engine.telemetry.addData("x pos", robot.posX);
        engine.telemetry.addData("y pos", robot.posY);
        engine.telemetry.addData("x pos target", robot.xTarget);
        engine.telemetry.addData("y pos target", robot.yTarget);
        engine.telemetry.addData("h pos odo", Math.toDegrees(robot.posH));
        engine.telemetry.addData("h pos odo target", Math.toDegrees(robot.hTarget));
        engine.telemetry.addData("h pos imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("input y pidPower", robot.pidX);
        engine.telemetry.addData("input x pidPower", robot.pidY);
        engine.telemetry.addData("raw x pid", robot.XPIDControl(robot.xTarget, robot.posX));
        engine.telemetry.addData("raw y pid", robot.YPIDControl(robot.yTarget, robot.posY));

    }
}
