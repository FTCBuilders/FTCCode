package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PIDOuttakeMotor {

    private final DcMotorEx motor;
    private double targetTicksPerSecond = 0;

    // Adjust these if needed
    private static final double achievableMaxTicksPerSecond = 1880.0;

    public PIDOuttakeMotor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder and enable RUN_USING_ENCODER mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure PIDF coefficients for velocity control
        double kF = 32767.0 / achievableMaxTicksPerSecond; // feedforward for max achievable
        PIDFCoefficients pidf = new PIDFCoefficients(0.001, 0.00005, 0.0, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    /** Start flywheel at desired velocity (ticks per second) */
    public void start(double ticksPerSecond) {
        if (ticksPerSecond < 0) ticksPerSecond = 0;
        if (ticksPerSecond > achievableMaxTicksPerSecond)
            ticksPerSecond = achievableMaxTicksPerSecond;

        targetTicksPerSecond = ticksPerSecond;
        motor.setVelocity(targetTicksPerSecond);
    }

    /** Stop the flywheel */
    public void stop() {
        targetTicksPerSecond = 0;
        motor.setPower(0);
    }

    /** Returns current velocity in ticks/sec */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** Returns the target velocity */
    public double getTargetVelocity() {
        return targetTicksPerSecond;
    }
}
