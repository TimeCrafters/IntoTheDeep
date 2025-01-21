package org.timecrafters.IntoTheDeep.autonomous.states;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.IntoTheDeep.common.Patriot;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class ArmSequencerState extends CyberarmState {

    Patriot robot;
    public String armPos;
    public boolean intakeOpen;
    public boolean depoOpen;
    private String actionName;

    public ArmSequencerState(Patriot robot, String groupName, String actionName) {
        this.actionName = actionName;

        this.robot = robot;
        this.armPos = robot.configuration.variable(groupName, actionName, "armPos").value();
        this.intakeOpen = robot.configuration.variable(groupName, actionName, "intakeOpen").value();
        this.depoOpen = robot.configuration.variable(groupName, actionName, "depoOpen").value();
    }

    @Override
    public void start() {
        super.start();
        robot.armPos = armPos;
        robot.autonomous = true;
        robot.intakeOpen = intakeOpen;
        robot.depoOpen = depoOpen;
    }

    @Override
    public void exec() {
        if (armPos.equals("Intake")){
            if (robot.rightIntakeDifInPos
                    && robot.leftIntakeDifInPos
                    && robot.intakeSlidesInPos){
                setHasFinished(true);
            }
        }
        if (armPos.equals("Deposit Basket")) {
            if (depoOpen){
                if (robot.depositSlidesInPos && robot.depositTimer){
                    setHasFinished(true);
                }
            } else {
                if (robot.depositSlidesInPos){
                    setHasFinished(true);
                }
            }
        }
        if (robot.rightIntakeDifInPos
                && robot.leftIntakeDifInPos
                && robot.intakeSlidesInPos
                && robot.depositSlidesInPos) {

            if (armPos.equals("Transfer 1/2")){
                if (robot.transferOneComplete){
                    setHasFinished(true);
                }
            } else if (armPos.equals("Transfer 2/2"))
                if (robot.transferTwoComplete) {
                    setHasFinished(true);
                }
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
        engine.telemetry.addData("Start Timer", robot.startTimer);
        engine.telemetry.addData("initial Time", robot.initialTime);
        engine.telemetry.addData("deposit Timer", robot.depositTimer);
        engine.telemetry.addData("h pos imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("leftIntakeDifInPos", robot.leftIntakeDifInPos);
        engine.telemetry.addData("rightIntakeDifInPos", robot.rightIntakeDifInPos);
        engine.telemetry.addData("rightDepositDifInPos", robot.rightIntakeDifInPos);
        engine.telemetry.addData("leftDepositDifInPos", robot.leftIntakeDifInPos);
        engine.telemetry.addData("slidesInPos", robot.depositSlidesInPos);
        engine.telemetry.addData("slidesInPos", robot.intakeSlidesInPos);
        engine.telemetry.addData("slidesInPos", robot.depositTarget);
        engine.telemetry.addData("slidesInPos", robot.depositRightExtendo.getCurrentPosition());
        engine.telemetry.addData("input y pidPower", robot.pidX);
        engine.telemetry.addData("input x pidPower", robot.pidY);
        engine.telemetry.addData("raw x pid", robot.XPIDControl(robot.xTarget, robot.posX));
        engine.telemetry.addData("raw y pid", robot.YPIDControl(robot.yTarget, robot.posY));
        engine.telemetry.addData("intake left diffy pos", robot.posIntakeLeftDiffy);
        engine.telemetry.addData("intake right diffy pos", robot.posIntakeRightDiffy);
        engine.telemetry.addData("depo left diffy pos", robot.posDepositLeftDiffy);
        engine.telemetry.addData("depo right diffy pos", robot.posDepositRightDiffy);
        engine.telemetry.addData("intake left difTarget", robot.leftIntakeDifTarget);
        engine.telemetry.addData("intake right difTarget", robot.rightIntakeDifTarget);
        engine.telemetry.addData("depo left difTarget", robot.leftDepositDifTarget);
        engine.telemetry.addData("depo right difTarget", robot.rightDepositDifTarget);

    }
}
