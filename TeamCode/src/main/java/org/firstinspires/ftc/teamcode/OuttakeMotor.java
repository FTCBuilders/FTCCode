package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeMotor {

    private final DcMotor motor;

    public OuttakeMotor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "outtakeMotor"); // same motor
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Intake direction (forward)
    public void intake(double power) {
        motor.setPower(Math.abs(power));  // always positive for intake
    }

    // Outtake direction (reverse)
    public void outtake(double power) {
        motor.setPower(-Math.abs(power)); // always negative for outtake
    }

    // Stop motor
    public void stop() {
        motor.setPower(0);
    }

    // Generic start (if you need direct control)
    public void setPower(double power) {
        motor.setPower(power);
    }

    // Get current power
    public double getPower(double v) {
        return motor.getPower();
    }
}
