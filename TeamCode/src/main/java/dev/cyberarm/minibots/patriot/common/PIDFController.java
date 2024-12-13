package dev.cyberarm.minibots.patriot.common;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    private double p = 0.0001, i = 0, d = 0, f = 0;
    private double maxI = 1.0;
    private double previousTime = 0, previousError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private DcMotorEx motor = null;
    private CRServoImplEx servo = null;
    public PIDFController(DcMotorEx motor) {
        this.motor = motor;
        this.timer.reset();
    }

    public PIDFController(CRServoImplEx servo) {
        this.servo = servo;
        this.timer.reset();
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public void update() {
        update(motor.getCurrentPosition(), motor.getTargetPosition());
    }

    public void update(int currentPosition, int targetPosition) {
        double currentTime = timer.milliseconds();
        int currentError = targetPosition - currentPosition;

        double p = this.p * currentError;
        double i = this.i * (currentError * (currentTime - previousTime));
        double d = this.d * (currentError - previousError) / (currentTime - previousTime);

        if (i > maxI) {
            i = maxI;
        } else if (i < -maxI) {
            i = -maxI;
        }

        double output = p + i + d + f;

        if (motor != null)
            motor.setPower(output);
        if (servo != null)
            servo.setPower(output);

        previousError = currentError;
        previousTime = currentTime;
    }
}
