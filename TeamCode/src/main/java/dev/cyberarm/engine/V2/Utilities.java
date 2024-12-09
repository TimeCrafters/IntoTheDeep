package dev.cyberarm.engine.V2;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Utilities {
    /***
     * The current heading of the robot as a full 360 degree value, instead of that half radian mess.
     * @param imu IMU of rev hub or similar
     * @param imuAngleOffset optional angle offset added to IMU value
     * @return full range heading of the robot, in DEGREES.
     */
    public static double facing(IMU imu, double imuAngleOffset) {
        double imuDegrees = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        return (((imuDegrees + 360.0) % 360.0) + imuAngleOffset) % 360.0;
    }

    public static double facing(IMU imu) {
        return facing(imu, 0);
    }

    public static double facing(double headingDegrees) {
        return (headingDegrees + 360.0) % 360.0;
    }

    public static double heading(double facing) {
        return AngleUnit.normalizeRadians(-facing * Math.PI / 180.0);
    }

    public static double turnRate(IMU imu) {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate;
    }

    /***
     *
     * @param value value to check with
     * @param min minimum value
     * @param max maximum value
     * @return true if value is between min and max. inclusive.
     */
    public static boolean isBetween(double value, double min, double max) {
        return value >= min && value <= max;
    }

    public static boolean isBetween(int value, int min, int max) {
        return value >= min && value <= max;
    }

    // Adapted from: https://github.com/gosu/gosu/blob/980d64de2ce52e4b16fdd5cb9c9e11c8bbb80671/src/Math.cpp#L38

    /***
     * The angular difference between two angles
     * **NOTE** flip flopped from and to values may result in continuous inversion of the angle difference (180 to -180 for example)
     * @param from angle in DEGREES
     * @param to angle in DEGREES
     * @return Angular difference two angles in DEGREES
     */
    public static double angleDiff(double from, double to) {
        double value = (to - from + 180);

        double fmod = (value - 0.0) % (360.0 - 0.0);

        return (fmod < 0 ? fmod + 360.0 : fmod +  0.0) - 180;
    }

    /***
     * Linear interpolation
     * @param min minimum value
     * @param max maximum value
     * @param t factor
     * @return
     */
    public static double lerp(double min, double max, double t)
    {
        return min + (max - min) * t;
    }

    /***
     * Calculates motor angle (degrees) in ticks
     * @param motorTicksPerRevolution
     * @param gearRatio
     * @param angleInDegrees
     * @return Angle in motor ticks
     */
    public static int motorAngleToTicks(int motorTicksPerRevolution, double gearRatio, double angleInDegrees) {
        double d = (gearRatio * motorTicksPerRevolution) / 360.0;

        return (int) (angleInDegrees * d);
    }

    /**
     * Calculates motor angle (degrees) from ticks
     * @param motorTicksPerRevolution
     * @param gearRatio
     * @param ticks
     * @return Motor ticks for angle
     */
    public static double motorTicksToAngle(int motorTicksPerRevolution, double gearRatio, int ticks) {
        double oneDegree = 360.0 / (gearRatio * motorTicksPerRevolution);

        return oneDegree * ticks;
    }

    /**
     * Convert a distance into motor ticks
     * @param motorTicksPerRevolution
     * @param gearRatio
     * @param wheelDiameterMM wheel diameter in millimeters
     * @param unit
     * @param distance
     * @return motor ticks for distance
     */
    public static int unitToTicks(int motorTicksPerRevolution, double gearRatio, double wheelDiameterMM, DistanceUnit unit, double distance) {
        double fMM = (gearRatio * motorTicksPerRevolution) / (wheelDiameterMM * Math.PI * (gearRatio * motorTicksPerRevolution) / (gearRatio * motorTicksPerRevolution));

        double mm = unit.toMm(unit.fromUnit(unit, distance));

        double ticks = fMM * mm;

        return (int)ticks;
    }

    /**
     * Convert ticks into a distance
     * @param motorTicksPerRevolution
     * @param gearRatio
     * @param wheelDiameterMM wheel diameter in millimeters
     * @param unit
     * @param ticks
     * @return distance for motor ticks
     */
    public static double ticksToUnit(int motorTicksPerRevolution, double gearRatio, double wheelDiameterMM, DistanceUnit unit, int ticks) {
        // Convert to millimeters, then to unit.
        double mm = wheelDiameterMM * Math.PI * ticks / (gearRatio * motorTicksPerRevolution);

        return unit.fromUnit(DistanceUnit.MM, mm);
    }

    public static boolean atTargetPosition(DcMotorEx motor) {
        return isBetween(
                motor.getCurrentPosition(),
                motor.getTargetPosition() - motor.getTargetPositionTolerance() / 2,
                motor.getTargetPosition() + motor.getTargetPositionTolerance() / 2);
    }

    /**
     * Sets bulk caching mode for hubs
     * @param hardwareMap
     * @param mode
     */
    public static void hubsBulkReadMode(HardwareMap hardwareMap, LynxModule.BulkCachingMode mode) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(mode);
        }
    }

    /**
     * Clears bulk cache data for hubs
     * @param hardwareMap
     */
    public static void hubsClearBulkReadCache(HardwareMap hardwareMap) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
    }

    /**
     * Get battery voltage
     * @param hardwareMap
     * @return battery voltage or POSITIVE_INFINITY if no voltage sensor is found or has an invalid value
     */
    public static double getVoltage(HardwareMap hardwareMap) {
        for (VoltageSensor voltageSensor : hardwareMap.voltageSensor) {
            double v = voltageSensor.getVoltage();

            if (v >= 6.0) {
                return voltageSensor.getVoltage();
            }
        }

        return Double.POSITIVE_INFINITY;
    }
}
