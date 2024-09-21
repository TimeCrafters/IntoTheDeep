package dev.cyberarm.minibots.voyager.teleop;

import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.voyager.common.VoyagerRobot;

public class VoyagerTeleOp extends CyberarmEngine {
    @Override
    public void setup() {
        setupFromConfig(
                new TimeCraftersConfiguration("Voyager"),
                "dev.cyberarm.minibots.voyager.teleop.states",
                new VoyagerRobot(this, false),
                VoyagerRobot.class,
                "TeleOp"
        );
    }
}
