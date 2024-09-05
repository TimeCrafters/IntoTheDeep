package org.timecrafters.diagnostics.rev_hub_test_suite;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.timecrafters.Library.Robot;
import org.timecrafters.diagnostics.rev_hub_test_suite.states.RevTestSuiteTestState;

import java.util.ArrayList;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class RevHubTestSuiteRobot implements Robot {
    final String TAG = "RevTestSuite|Robot";

    private HardwareMap hardwareMap;
    public boolean testingControlHub = true;
    public RevTestSuiteTestState.STAGE stage = RevTestSuiteTestState.STAGE.NONE;
    public final ArrayList<String> reports = new ArrayList<>();
    public ArrayList<DcMotorEx> controlHubMotors = new ArrayList<>(), expansionHubMotors = new ArrayList<>();
    public ArrayList<Servo> controlHubServos = new ArrayList<>(), expansionHubServos = new ArrayList<>();
    public ArrayList<AnalogInput> controlHubAnalogSensors = new ArrayList<>(), expansionHubAnalogSensors = new ArrayList<>();
    public ArrayList<DigitalChannel> controlHubDigitalSensors = new ArrayList<>(), expansionHubDigitalSensors = new ArrayList<>();
    public ArrayList<Rev2mDistanceSensor> controlHubI2cSensors = new ArrayList<>(), expansionHubI2cSensors = new ArrayList<>();

    public ArrayList<LynxModule> lynxModules = new ArrayList<>();
    @Override
    public void setup() {
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;

        this.lynxModules = new ArrayList<>(hardwareMap.getAll(LynxModule.class));

        /* ------------------------------------------------ Control Hub Devices ------------------------------------------------ */
        // MOTORS
        controlHubMotors.add((DcMotorEx) hardwareMap.dcMotor.get("c_motor_0"));
        controlHubMotors.add((DcMotorEx) hardwareMap.dcMotor.get("c_motor_1"));
        controlHubMotors.add((DcMotorEx) hardwareMap.dcMotor.get("c_motor_2"));
        controlHubMotors.add((DcMotorEx) hardwareMap.dcMotor.get("c_motor_3"));

//        // SERVOS
        controlHubServos.add(hardwareMap.servo.get("c_servo_1"));
        controlHubServos.add(hardwareMap.servo.get("c_servo_2"));
        controlHubServos.add(hardwareMap.servo.get("c_servo_3"));
        controlHubServos.add(hardwareMap.servo.get("c_servo_4"));
        controlHubServos.add(hardwareMap.servo.get("c_servo_5"));
        controlHubServos.add(hardwareMap.servo.get("c_servo_0"));

        // ANALOG SENSORS
        controlHubAnalogSensors.add(hardwareMap.analogInput.get("c_analog_0"));
        controlHubAnalogSensors.add(hardwareMap.analogInput.get("c_analog_1"));
        controlHubAnalogSensors.add(hardwareMap.analogInput.get("c_analog_2"));
        controlHubAnalogSensors.add(hardwareMap.analogInput.get("c_analog_3"));

        // DIGITAL SENSORS
        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_0"));
        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_1"));
        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_2"));
        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_3"));
        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_4"));
        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_5"));
        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_6"));
        controlHubDigitalSensors.add(hardwareMap.digitalChannel.get("c_digital_7"));

        // I2C SENSORS
        controlHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "c_i2c_0"));
        controlHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "c_i2c_1"));
        controlHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "c_i2c_2"));
        controlHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "c_i2c_3"));

//        /* ------------------------------------------------ Expansion Hub Devices ------------------------------------------------ */
//        // MOTORS
        expansionHubMotors.add((DcMotorEx) hardwareMap.dcMotor.get("x_motor_0"));
        expansionHubMotors.add((DcMotorEx) hardwareMap.dcMotor.get("x_motor_1"));
        expansionHubMotors.add((DcMotorEx) hardwareMap.dcMotor.get("x_motor_2"));
        expansionHubMotors.add((DcMotorEx) hardwareMap.dcMotor.get("x_motor_3"));
//
//        // SERVOS
        expansionHubServos.add(hardwareMap.servo.get("x_servo_0"));
        expansionHubServos.add(hardwareMap.servo.get("x_servo_1"));
        expansionHubServos.add(hardwareMap.servo.get("x_servo_2"));
        expansionHubServos.add(hardwareMap.servo.get("x_servo_3"));
        expansionHubServos.add(hardwareMap.servo.get("x_servo_4"));
        expansionHubServos.add(hardwareMap.servo.get("x_servo_5"));
//
//        // ANALOG SENSORS
        expansionHubAnalogSensors.add(hardwareMap.analogInput.get("x_analog_0"));
        expansionHubAnalogSensors.add(hardwareMap.analogInput.get("x_analog_1"));
        expansionHubAnalogSensors.add(hardwareMap.analogInput.get("x_analog_2"));
        expansionHubAnalogSensors.add(hardwareMap.analogInput.get("x_analog_3"));
//
//        // DIGITAL SENSORS
        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_0"));
        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_1"));
        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_2"));
        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_3"));
        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_4"));
        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_5"));
        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_6"));
        expansionHubDigitalSensors.add(hardwareMap.digitalChannel.get("x_digital_7"));
//
//        // I2C SENSORS
        expansionHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "x_i2c_0"));
        expansionHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "x_i2c_1"));
        expansionHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "x_i2c_2"));
        expansionHubI2cSensors.add(hardwareMap.get(Rev2mDistanceSensor.class, "x_i2c_3"));
//
//        /* ------------------------------------------------ Hub Sensor Reading Optimization ------------------------------------------------ */
        for (LynxModule hub : lynxModules) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        /* ------------------------------------------------ Motor Configuration ------------------------------------------------ */
        configureMotors();

        /* ------------------------------------------------ Digital Sensor Configuration ------------------------------------------------ */
        configureDigitalSensors();
    }

    protected void clearStaleData() {
        for (LynxModule hub : lynxModules) {
            hub.clearBulkCache();
        }
    }

    private void configureMotors() {
        for(DcMotorEx motor : controlHubMotors) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for(DcMotorEx motor : expansionHubMotors) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private void configureDigitalSensors() {
        for(DigitalChannel sensor : controlHubDigitalSensors) {
            sensor.setMode(DigitalChannel.Mode.INPUT);
        }

        for(DigitalChannel sensor : expansionHubDigitalSensors) {
            sensor.setMode(DigitalChannel.Mode.INPUT);
        }
    }
}
