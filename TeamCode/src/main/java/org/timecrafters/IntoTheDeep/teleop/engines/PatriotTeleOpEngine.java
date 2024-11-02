package org.timecrafters.IntoTheDeep.teleop.engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.IntoTheDeep.common.Patriot;
import org.timecrafters.IntoTheDeep.teleop.states.PatriotTeleOpState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Patriot TeleOp", group = "0 Competition V1")
public class PatriotTeleOpEngine extends CyberarmEngine {
    private Patriot robot;
    @Override
    public void setup() {
        this.robot = new Patriot("Hello World");
        this.robot.setup();

        addState(new PatriotTeleOpState(robot));
    }
}
