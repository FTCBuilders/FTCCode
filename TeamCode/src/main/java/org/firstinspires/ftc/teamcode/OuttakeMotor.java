package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class OuttakeMotor {

    private final DcMotorEx motor;
    private final VoltageSensor voltageSensor;

    // PID constants for outtake
    private double kP = 0.0005;
    private double kI = 0.00001;
    private double kD = 0.0001;

    private double integral = 0;
    private double lastError = 0;

    private double nominalVoltage = 12.0;
    private long lastTime;

    public OuttakeMotor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        lastTime = System.nanoTime();
    }

    // ===== Intake control (stronger, ignores PID) =====
    public void intake(double power) {
        // boost intake without affecting PID for outtake
        motor.setPower(Math.min(Math.abs(power) * 1.5, 1.0)); // clamp to 1
    }

    // ===== Outtake control (always PID) =====
    public void outtake(double power) {
        setVelocity(-Math.abs(power) * 500); // PID-controlled
    }

    // Stop motor
    public void stop() {
        motor.setPower(0);
        resetPID();
    }

    // Reset PID terms
    public void resetPID() {
        integral = 0;
        lastError = 0;
    }

    // PID velocity control with battery compensation
    protected void setVelocity(double targetVelocity) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1e9; // seconds
        lastTime = currentTime;

        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;

        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        double pidPower = kP * error + kI * integral + kD * derivative;

        // Battery compensation
        double voltageFactor = nominalVoltage / voltageSensor.getVoltage();
        double adjustedPower = pidPower * voltageFactor;

        // Clamp power
        adjustedPower = Math.max(-1, Math.min(1, adjustedPower));

        motor.setPower(adjustedPower);
    }

    public double getCurrentVelocity() {
        return motor.getVelocity();
    }
}
