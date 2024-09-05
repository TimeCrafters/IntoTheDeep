package dev.cyberarm.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.cyberarm.drivers.EncoderCustomKB2040;
import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.CyberarmState;

//@TeleOp(name = "I2C Driver Test", group = "TESTING")
public class CustomEncoderTestEngine extends CyberarmEngine {
    @Override
    public void setup() {
        addState(new CyberarmState() {
            private EncoderCustomKB2040 encoder;
            private double triggerMS = 0;
            private int position = -1;

            public void init() {
                encoder = engine.hardwareMap.get(EncoderCustomKB2040.class, "encoder");
            }

            @Override
            public void exec() {
                if (runTime() - triggerMS >= 0) {
                    triggerMS = runTime();

                    position = encoder.getCurrentPosition();
//                    encoder.reset();
                }
            }

            @Override
            public void telemetry() {
                engine.telemetry.addData("POS", position);
                engine.telemetry.addData("RunTime", runTime());
                engine.telemetry.addData("Trigger", triggerMS);
            }
        });
    }
}
