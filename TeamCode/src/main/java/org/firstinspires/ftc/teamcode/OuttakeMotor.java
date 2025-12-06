package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class OuttakeMotor {

    private final DcMotorEx motor;
    private double targetVelocity = 0;

    // Estimated max ticks per second (change if you know your motor specs)
    private static final double MAX_TICKS_PER_SECOND = 1880.0;

    // PIDF values for built-in control — tune if needed
    private static final double kP = 1.0;
    private static final double kI = 0.00005;
    private static final double kD = 0.0;
    private static final double kF = 17.0; // Feedforward, scales motor power

    public OuttakeMotor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        // Basic setup
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply PIDF coefficients
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    /** Start the flywheel with desired velocity (ticks/second) */
    public void start(double ticksPerSecond) {
        if (ticksPerSecond < 0) ticksPerSecond = 0;
        if (ticksPerSecond > MAX_TICKS_PER_SECOND)
            ticksPerSecond = MAX_TICKS_PER_SECOND;

        targetVelocity = ticksPerSecond;
        motor.setVelocity(targetVelocity);
    }

    /** Stop the flywheel completely */
    public void stop() {
        targetVelocity = 0;
        motor.setPower(0);
    }

    /** Update method (optional if velocity already being held) */
    public void update() {
        if (targetVelocity > 0) {
            motor.setVelocity(targetVelocity);
        } else {
            motor.setPower(0);
        }
    }

    /** Return the flywheel's current speed (ticks/sec) */
    public double getCurrentVelocity() {
        return motor.getVelocity();
    }

    /** Return the target velocity */
    public double getTargetVelocity() {
        return targetVelocity;
    }
}
