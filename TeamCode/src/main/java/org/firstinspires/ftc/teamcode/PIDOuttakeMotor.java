package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDOuttakeMotor {

    private final DcMotorEx outtakeMotor;

    // --- PIDF coefficients (starting points) ---
    private static final double kP = 30.0; // acceleration speed for getting to targeted rpm
    private static final double kI = 0.0; // patches up a small, permanent error
    private static final double kD = 0.0; // slows down acceleration just before targeted rpm
    private static final double kF = 0.85; // base power

    // --- Desired velocity ---
    private double targetTicksPerSecond = 0;

    public PIDOuttakeMotor(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    // Run motor at desired velocity
    public void start(double ticksPerSecond) {
        targetTicksPerSecond = ticksPerSecond;
        outtakeMotor.setVelocity(ticksPerSecond);
    }

    // Stop motor
    public void stop() {
        targetTicksPerSecond = 0;
        outtakeMotor.setPower(0);
    }

    public double getVelocity() {
        return outtakeMotor.getVelocity();
    }

    public double getTargetVelocity() {
        return targetTicksPerSecond;
    }
}
