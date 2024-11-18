package org.timecrafters.IntoTheDeep.common;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.timecrafters.Library.Robot;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class Patriot implements Robot {

    private HardwareMap hardwareMap;
    private String string;
    private CyberarmEngine engine;
    public DcMotorEx fl, fr, bl, br, intakeExtendo, depositRightExtendo, depositLeftExtendo;
    public CRServo leftDiff, rightDiff;
    public Servo depoClaw, depoLeft, depoRight, intakeClaw;
    public final int RIGHT_DIFFY = 1;
    public final int LEFT_DIFFY  = 0;

    // Declare the OctoQuad object and members to store encoder positions and velocities
    public OctoQuad octoquad;
    public int posLeftDiffy;
    public int posRightDiffy;
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

        // OCTOQUAD
        octoquad = engine.hardwareMap.get(OctoQuad.class, "octoquad");
        octoquad.setSingleEncoderDirection(LEFT_DIFFY,  OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(RIGHT_DIFFY, OctoQuad.EncoderDirection.REVERSE);

        //motors configuration
        intakeExtendo = (DcMotorEx) engine.hardwareMap.dcMotor.get("extension");
        depositLeftExtendo = (DcMotorEx) engine.hardwareMap.dcMotor.get ("leftLift");
        depositRightExtendo = (DcMotorEx) engine.hardwareMap.dcMotor.get ("rightLift");

        depositLeftExtendo.setDirection(DcMotorSimple.Direction.REVERSE);


        fl = (DcMotorEx) engine.hardwareMap.dcMotor.get("fl");
        fr = (DcMotorEx) engine.hardwareMap.dcMotor.get("fr");
        br = (DcMotorEx) engine.hardwareMap.dcMotor.get("br");
        bl = (DcMotorEx) engine.hardwareMap.dcMotor.get("bl");

        fl.setDirection(DcMotorEx.Direction.FORWARD);
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fr.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        br.setDirection(DcMotorEx.Direction.REVERSE);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));


        imu.initialize(parameters);

        // SERVO'S
        intakeClaw = engine.hardwareMap.servo.get("theClaw");
        depoClaw = engine.hardwareMap.servo.get("depo claw");
        depoLeft = engine.hardwareMap.servo.get("depo left");
        depoRight = engine.hardwareMap.servo.get("depo right");
        depoRight.setDirection(Servo.Direction.FORWARD);
        depoLeft.setDirection(Servo.Direction.REVERSE);

        // CR SERVO'S
        leftDiff = engine.hardwareMap.crservo.get("leftDiff");
        rightDiff = engine.hardwareMap.crservo.get("rightDiff");
        leftDiff.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDiff.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void readOctoQuad(){
        int[] positions = octoquad.readAllPositions();
        posLeftDiffy  = positions[LEFT_DIFFY];
        posRightDiffy = positions[RIGHT_DIFFY];
    }
}
