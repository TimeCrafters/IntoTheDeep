package org.timecrafters.IntoTheDeep.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Config
public class PatriotMk2 implements Robot {
    private HardwareMap hardwareMap;
    public TimeCraftersConfiguration configuration;
    private String string;
    private CyberarmEngine engine;
    public DcMotorEx fl, fr, bl, br;
    public IMU imu;

    // heading control
    public double headingIntegralSum = 0;
    public static double Hp = 0.7, Hi = 0, Hd = 0;
    private double headingLastError = 0;
    ElapsedTime headingTimer = new ElapsedTime();

    public PatriotMk2(String string) {
        this.engine = engine;
        setup();
        this.string = string;
    }

    public void setup() {
        System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;

        imu = engine.hardwareMap.get(IMU.class, "imu_ex");

        fl = (DcMotorEx) engine.hardwareMap.dcMotor.get("fl");
        fr = (DcMotorEx) engine.hardwareMap.dcMotor.get("fr");
        br = (DcMotorEx) engine.hardwareMap.dcMotor.get("br");
        bl = (DcMotorEx) engine.hardwareMap.dcMotor.get("bl");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fr.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        br.setDirection(DcMotorEx.Direction.FORWARD);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public double HeadingPIDControl(double reference, double current) {
        double error = angleWrap(current - reference);
        headingIntegralSum += error * headingTimer.seconds();
        double derivative = (error - headingLastError) / headingTimer.seconds();

        headingTimer.reset();

        double output = (error * Hp) + (derivative * Hd) + (headingIntegralSum * Hi);
        return output;
    }

}
