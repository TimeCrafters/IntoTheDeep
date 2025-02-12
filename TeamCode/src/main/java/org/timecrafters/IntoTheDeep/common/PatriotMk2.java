package org.timecrafters.IntoTheDeep.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Config
public class PatriotMk2 implements Robot {
    private HardwareMap hardwareMap;
    public TimeCraftersConfiguration configuration;
    public String TeleOrAuto = "tele";
    private String string;
    private CyberarmEngine engine;

    // --------------------------------------------------------
    // ----------------- Hardware Map -------------------------
    // --------------------------------------------------------
    public DcMotorEx fl, fr, bl, br;
    public DcMotorEx pivot, extensionLeft, extensionRight;

    public IMU imu;

    public Servo leftPivot, rightPivot, twist, claw;

    // --------------------------------------------------------
    // -------------- PID Controllers -------------------------
    // --------------------------------------------------------


    // -------------- Heading Control ------------------
    public double headingIntegralSum = 0;
    public static double Hp = 1, Hi = 0, Hd = 0;
    private double headingLastError = 0;
    ElapsedTime headingTimer = new ElapsedTime();

    // X Axis Control ----------------------------------
    public double xAxisIntegralSum = 0;
    public static double Xp = 0.7, Xi = 0, Xd = 0;
    private double xAxisLastError = 0;
    ElapsedTime xAxisTimer = new ElapsedTime();

    // Y Axis Control ----------------------------------
    public double yAxisIntegralSum = 0;
    public static double Yp = 0.7, Yi = 0, Yd = 0;
    private double yAxisLastError = 0;
    ElapsedTime yAxisTimer = new ElapsedTime();

    // Extension Control -------------------------------
    public int intakeFar = -600;
    public int intakeClose = -100;
    public int stow = 0;
    public int basketHigh = -750;
    public int basketLow = -300;

    public double extensionIntegralSum = 0;
    public static double Ep = 0.005, Ei = 0, Ed = 0;
    private double extensionLastError = 0;
    ElapsedTime extensionTimer = new ElapsedTime();

    // Pivot Control -----------------------------------
    public double pivotIntaking = -50;
    public double pivotDepositing = 1200;
    public double pivotMotorHover = 0;
    public double pivotIntegralSum = 0;
    public static double Pp = 0.00085, Pi = 0, Pd = 0;
    private double pivotLastError = 0;
    ElapsedTime pivotTimer = new ElapsedTime();

    // ----------------------------------------------------------
    // Automation Variables for arm -----------------------------
    // ----------------------------------------------------------

    public String armMode = "stowed";
    public String servoMode = "stowed";
    public double variableWristAngle;
    public int oneTimeDeal = 0;
    public long initTime = 0;
    public long pickDelay = 0;
    public double twistOrigin = 0.49;
    public double twistPos = 0.49;
    public double pivotPos = 0.6;
    public double clawPos = 0.37;
    public double pivotBasket = 0.6;
    public double pivotBasketOut = 0.6;
    public double pivotHover = 0.4;
    public double pivotIntake = 0.15;
    public double pivotStow = 0.6;
    public double clawOpen = 0;
    public double clawClose = 0.37;

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

        pivot = (DcMotorEx) engine.hardwareMap.dcMotor.get("pivot");

        extensionLeft = (DcMotorEx) engine.hardwareMap.dcMotor.get("sbl");
        extensionRight = (DcMotorEx) engine.hardwareMap.dcMotor.get("sbr");

        leftPivot = engine.hardwareMap.servo.get("lp");
        rightPivot = engine.hardwareMap.servo.get("rp");
        twist = engine.hardwareMap.servo.get("twist");
        claw = engine.hardwareMap.servo.get("claw");

        leftPivot.setDirection(Servo.Direction.FORWARD);
        rightPivot.setDirection(Servo.Direction.REVERSE);

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        bl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        br.setDirection(DcMotorEx.Direction.FORWARD);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot.setDirection(DcMotorEx.Direction.REVERSE);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionRight.setDirection(DcMotorEx.Direction.REVERSE);
        extensionRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extensionRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionLeft.setDirection(DcMotorEx.Direction.REVERSE);
        extensionLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extensionLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

    // -------------------------------------------------------------
    // ------- PID Controllers for all automated functions ---------
    // -------------------------------------------------------------

    public double HeadingPIDControl(double target, double current) {
        double error = angleWrap(current - target);
        headingIntegralSum += error * headingTimer.seconds();
        double derivative = (error - headingLastError) / headingTimer.seconds();

        headingTimer.reset();

        double output = (error * Hp) + (derivative * Hd) + (headingIntegralSum * Hi);
        return output;
    }


    public double XAxisPIDControl(double target, double current) {
        double error = current - target;
        xAxisIntegralSum += error * xAxisTimer.seconds();
        double derivative = (error - xAxisLastError) / xAxisTimer.seconds();

        xAxisTimer.reset();

        double output = (error * Xp) + (derivative * Xd) + (xAxisIntegralSum * Xi);
        return output;
    }

    public double YAxisPIDControl(double target, double current) {
        double error = current - target;
        yAxisIntegralSum += error * yAxisTimer.seconds();
        double derivative = (error - yAxisLastError) / yAxisTimer.seconds();

        yAxisTimer.reset();

        double output = (error * Yp) + (derivative * Yd) + (yAxisIntegralSum * Yi);
        return output;
    }

    public double ExtensionPIDControl(double target, double current) {
        double error = current - target;
        extensionIntegralSum += error * extensionTimer.seconds();
        double derivative = (error - extensionLastError) / extensionTimer.seconds();

        extensionTimer.reset();

        double output = (error * Ep) + (derivative * Ed) + (extensionIntegralSum * Ei);
        return output;
    }

    public double PivotPIDControl(double target, double current) {
        double error = current - target;
        pivotIntegralSum += error * pivotTimer.seconds();
        double derivative = (error - pivotLastError) / pivotTimer.seconds();

        pivotTimer.reset();

        double output = (error * Pp) + (derivative * Pd) + (pivotIntegralSum * Pi);
        return output;
    }


    // -------------------------------------------------------------
    // ------- Arm Automation Modes And Control Presets ------------
    // -------------------------------------------------------------

    public void ArmModesPreset() {

        int currentExtensionPos = extensionLeft.getCurrentPosition();
        int currentPivotPos = pivot.getCurrentPosition();

        switch (armMode) {
            case "stowed":
                extensionLeft.setPower(-ExtensionPIDControl(stow, currentExtensionPos));
                extensionRight.setPower(-ExtensionPIDControl(stow, currentExtensionPos));
                if (extensionLeft.getCurrentPosition() > -200) {
                    pivot.setPower(PivotPIDControl(pivotIntaking, currentPivotPos));
                }

                break;
            case "intake-close":
                extensionLeft.setPower(-ExtensionPIDControl(intakeClose, currentExtensionPos));
                extensionRight.setPower(-ExtensionPIDControl(intakeClose, currentExtensionPos));
                pivot.setPower(PivotPIDControl(pivotIntaking, currentPivotPos));


                break;
            case "intake-far-hover":
                extensionLeft.setPower(-ExtensionPIDControl(intakeFar, currentExtensionPos));
                extensionRight.setPower(-ExtensionPIDControl(intakeFar, currentExtensionPos));
                pivot.setPower(PivotPIDControl(pivotMotorHover, currentPivotPos));

                break;
            case "intake-far":
                extensionLeft.setPower(-ExtensionPIDControl(intakeFar, currentExtensionPos));
                extensionRight.setPower(-ExtensionPIDControl(intakeFar, currentExtensionPos));
                pivot.setPower(PivotPIDControl(pivotIntaking, currentPivotPos));

                break;
            case "basket-high":
                pivot.setPower(PivotPIDControl(pivotDepositing, currentPivotPos));
                if (pivot.getCurrentPosition() > pivotDepositing - 300){
                    extensionLeft.setPower(-ExtensionPIDControl(basketHigh, currentExtensionPos));
                    extensionRight.setPower(-ExtensionPIDControl(basketHigh, currentExtensionPos));
                }

                break;
            case "basket-low":
                pivot.setPower(PivotPIDControl(pivotDepositing, currentPivotPos));
                if (pivot.getCurrentPosition() > pivotDepositing - 300){
                    extensionLeft.setPower(-ExtensionPIDControl(basketLow, currentExtensionPos));
                    extensionRight.setPower(-ExtensionPIDControl(basketLow, currentExtensionPos));
                }

                break;
        }
    }

    public void ServoModePreset() {

        switch (servoMode) {
            case "basket":
                pivotPos = pivotBasket;
                clawPos = clawClose;
                twistPos = twistOrigin;
                break;
            case  "basket-out":
                pivotPos = pivotBasketOut;
                clawPos = clawOpen;
                twistPos = twistOrigin;
                break;
            case "stowed-from-basket":
                pivotPos = pivotStow;
                clawPos = clawOpen;
                twistPos = twistOrigin;
                break;
            case "pickup-far-hover-before":
                pivotPos = pivotHover;
                clawPos = clawOpen;
                twistPos = twistOrigin;
                break;
            case "pickup-aim":
                pivotPos = pivotIntake;
                clawPos = clawOpen;

                // TODO: 2/11/2025 Test this because it may not actually work.
                if (TeleOrAuto.equals("tele")) {
                    if (engine.gamepad1.x) {
                        twistPos = twistOrigin;
                    }

                    if (engine.gamepad1.right_bumper) {
                        twistPos = twistOrigin + 0.1;
                    } else if (engine.gamepad1.left_bumper) {
                        twistPos = twistOrigin - 0.1;
                    }
                } else if (TeleOrAuto.equals("auto")){
                    twistPos = variableWristAngle;
                }
                oneTimeDeal = 0;

                break;
            case "pickup-shoot":

                    clawPos = clawClose;

                    if (oneTimeDeal == 0) {
                        initTime = System.currentTimeMillis();
                        oneTimeDeal += 1;
                    }

                    if (initTime + pickDelay < System.currentTimeMillis()) {
                        servoMode = "pickup-far-hover-after";
                    }

                // TODO: 2/11/2025 Test this because quick release may not work.

                break;
            case "pickup-far-hover-after":
                pivotPos = pivotHover;
                clawPos = clawClose;
                twistPos = twistOrigin;

                oneTimeDeal = 0;

                break;
            case "stowed-from-pickup":

                pivotPos = pivotStow;
                clawPos = clawClose;
                twistPos = twistOrigin;
                break;

        }
    }
}
