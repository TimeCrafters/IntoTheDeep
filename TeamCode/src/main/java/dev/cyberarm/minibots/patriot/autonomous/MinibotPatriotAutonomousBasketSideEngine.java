package dev.cyberarm.minibots.patriot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobotTask;

@Autonomous(name = "Minibot Patriot Mk 1 - Basket", group = "minibot", preselectTeleOp = "Minibot Patriot Mk 1 - TeleOp")
public class MinibotPatriotAutonomousBasketSideEngine extends CyberarmEngine {
    @Override
    public void setup() {
        final MinibotPatriotRobot robot = new MinibotPatriotRobot(this, true);

        addTask(new MinibotPatriotRobotTask(robot));

        setupFromConfig(
                robot.config,
                "dev.cyberarm.minibots.patriot.autonomous.states",
                robot,
                MinibotPatriotRobot.class,
                "AutonomousBasketSide");
    }
}
