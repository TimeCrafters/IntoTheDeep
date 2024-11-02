package org.timecrafters.IntoTheDeep.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.timecrafters.Library.Robot;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class Patriot implements Robot {

    private HardwareMap hardwareMap;
    private String string;
    private CyberarmEngine engine;
    public DcMotor fl, fr, bl, br;
    public IMU imu;

    public Patriot(String string) {
        this.engine = engine;
        setup();
        this.string = string;
    }

    public void setup() {
        System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;
        //      IMU
        imu = engine.hardwareMap.get(IMU.class, "imu_ex");

        //motors configuration
        fl = engine.hardwareMap.dcMotor.get("fl");
        fr = engine.hardwareMap.dcMotor.get("fr");
        br = engine.hardwareMap.dcMotor.get("br");
        bl = engine.hardwareMap.dcMotor.get("bl");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));


        imu.initialize(parameters);

    }
}
