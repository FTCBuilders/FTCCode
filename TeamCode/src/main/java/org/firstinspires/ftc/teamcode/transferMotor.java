package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class transferMotor {
    private DcMotor transferMotor;

    public transferMotor(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // moves forward
    public void setPower(double power) {
        transferMotor.setPower(power);
    }

    // Stop motor
    public void stop() {
        transferMotor.setPower(0);
    }

    // Get current motor power
    public double getPower() {
        return transferMotor.getPower();
    }



}
