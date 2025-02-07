package dev.cyberarm.scratchpad;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;

@TeleOp(name = "INIT LOOP TEST", group = "testing")
public class InitLoopOpMode extends CyberarmEngine {
    @Override
    public void setup() {
        addInitState(new CyberarmState() {
            private Servo servo;
            double counter = 0;
            boolean up = true;
            double last_milliseconds = 0;
            @Override
            public void init() {
                servo = engine.hardwareMap.servo.get("servo");
            }

            @Override
            public void exec() {
                double dt = runTime() - last_milliseconds;
                if (dt > 1 / 60.0)
                    dt = 1 / 60.0;

                if (up) {
                    counter += dt;

                    if (counter >= 5.0)
                        up = false;
                } else {
                    counter -= dt;

                    if (counter <= 0.0)
                        up = true;
                }


                double i = (counter) / 5.0;
                // Quick and dirty sweep
                servo.setPosition(Utilities.lerp(0.0, 1.0, i));

                last_milliseconds = runTime();
            }

            @Override
            public void buttonDown(Gamepad gamepad, String button) {
                engine.telemetry.addData(gamepad.toString(), "%s is down", button);
            }

            @Override
            public void buttonUp(Gamepad gamepad, String button) {
                engine.telemetry.addData(gamepad.toString(), "%s is up", button);
            }
        });

        addState(new CyberarmState() {
            private Servo servo;
            @Override
            public void init() {
                servo = engine.hardwareMap.servo.get("servo");
            }

            @Override
            public void exec() {
                servo.setPosition(0);
            }
        });
    }
}
