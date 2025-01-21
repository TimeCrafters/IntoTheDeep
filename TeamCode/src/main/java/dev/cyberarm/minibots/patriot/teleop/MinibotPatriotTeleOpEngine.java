package dev.cyberarm.minibots.patriot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobot;
import dev.cyberarm.minibots.patriot.common.MinibotPatriotRobotTask;

@TeleOp(name = "Minibot Patriot Mk 1 - TeleOp", group = "minibot")
public class MinibotPatriotTeleOpEngine extends CyberarmEngine {
    @Override
    public void setup() {
        final MinibotPatriotRobot robot;
        if (MinibotPatriotRobot.getInstance() != null) {
            robot = MinibotPatriotRobot.getInstance();
        } else {
            robot = new MinibotPatriotRobot(this, false);
        }

        robot.setTeleOp(this);

        addTask(new MinibotPatriotRobotTask(robot));

        addState(new MinibotPatriotTeleOpState(robot, "TeleOp", "00-00"));
    }
}
