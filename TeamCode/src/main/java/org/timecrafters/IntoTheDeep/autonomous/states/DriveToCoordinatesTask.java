package org.timecrafters.IntoTheDeep.autonomous.states;

import com.acmerobotics.dashboard.config.Config;

import org.timecrafters.IntoTheDeep.common.Patriot;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class DriveToCoordinatesTask extends CyberarmState {

    Patriot robot;
    public DriveToCoordinatesTask(Patriot robot) {this.robot = robot;}
    @Override
    public void exec() {
        robot.HorizontalExtendoControl();
        robot.OdoLocalizer();
        robot.XDrivePowerModifier();
        robot.YDrivePowerModifier();
        robot.HDrivePowerModifier();
        robot.DriveToCoordinates();
        robot.readOctoQuad();
        robot.ArmSequencer();
        robot.HorizontalExtendoControl();
        robot.DepoExtendoControl();


    }
}
