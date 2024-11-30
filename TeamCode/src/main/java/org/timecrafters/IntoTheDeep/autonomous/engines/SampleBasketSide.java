package org.timecrafters.IntoTheDeep.autonomous.engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.IntoTheDeep.autonomous.states.DriveToCoordinatesTask;
import org.timecrafters.IntoTheDeep.common.Patriot;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "Sample Basket Side", preselectTeleOp = "qualifier 1")

public class SampleBasketSide extends CyberarmEngine {

    Patriot robot;


    @Override
    public void init() {
        super.init();
        robot.imu.resetYaw();
    }

    @Override
    public void setup() {
        this.robot = new Patriot("SampleBasketSide");
        addTask(new DriveToCoordinatesTask(robot));
        this.robot.setup();
        setupFromConfig(robot.configuration, "org.timecrafters.IntoTheDeep.autonomous.states", robot, Patriot.class, "SampleBasketSide");











    }

}
