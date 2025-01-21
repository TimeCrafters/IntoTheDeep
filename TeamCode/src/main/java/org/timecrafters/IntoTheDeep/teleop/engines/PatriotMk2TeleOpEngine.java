package org.timecrafters.IntoTheDeep.teleop.engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.IntoTheDeep.common.PatriotMk2;
import org.timecrafters.IntoTheDeep.teleop.states.PatriotMk2TeleOpState;
import org.timecrafters.IntoTheDeep.teleop.states.PatriotTeleOpState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Patriot State TeleOp", group = "0 Competition V1")
public class PatriotMk2TeleOpEngine extends CyberarmEngine {
    private PatriotMk2 robot;
    @Override
    public void setup() {
        this.robot = new PatriotMk2("Hello World");
        this.robot.setup();

        addState(new PatriotMk2TeleOpState(robot));
    }
}
