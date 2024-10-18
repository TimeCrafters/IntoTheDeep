package org.timecrafters.IntoTheDeep.teleop.engines;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.IntoTheDeep.teleop.states.TeleOpState;

import dev.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "ROBOT")
public class TeleOpEngine extends CyberarmEngine {
    @Override
    public void setup() {
        addState(new TeleOpState());
    }
}
