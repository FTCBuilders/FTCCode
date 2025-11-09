package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PIDOuttakeMotor {

    private final DcMotorEx motor;
    private double targetTicksPerSecond = 0;

    private static final double ACHIEVABLE_MAX_TICKS_PER_SECOND = 1880.0;

    public PIDOuttakeMotor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset encoder and enable velocity mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for velocity control
        double kF = 32767.0 / ACHIEVABLE_MAX_TICKS_PER_SECOND; // max feedforward
        PIDFCoefficients pidf = new PIDFCoefficients(0.001, 0.00005, 0.0, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    /** Start flywheel at desired velocity */
    public void start(double ticksPerSecond) {
        if (ticksPerSecond < 0) ticksPerSecond = 0;
        if (ticksPerSecond > ACHIEVABLE_MAX_TICKS_PER_SECOND)
            ticksPerSecond = ACHIEVABLE_MAX_TICKS_PER_SECOND;

        targetTicksPerSecond = ticksPerSecond;
    }

    /** Stop the flywheel */
    public void stop() {
        targetTicksPerSecond = 0;
        motor.setPower(0);
    }

    /** Call this in your loop() every tick to enforce velocity */
    public void update() {
        if (targetTicksPerSecond > 0) {
            motor.setVelocity(targetTicksPerSecond);
        } else {
            motor.setPower(0);
        }
    }

    /** Returns current velocity in ticks/sec */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** Returns target velocity */
    public double getTargetVelocity() {
        return targetTicksPerSecond;
    }
}
