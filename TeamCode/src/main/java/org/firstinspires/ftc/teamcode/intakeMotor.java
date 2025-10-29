package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class intakeMotor {

    private final DcMotor motor;


    public intakeMotor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Set motor power directly (-1.0 to 1.0) */
    public void setPower(double power) {
        motor.setPower(power);
    }

    /** Stop motor */
    public void stop() {
        motor.setPower(0);
    }

    /** Get current motor power */
    public double getPower() {
        return motor.getPower();
    }



}
