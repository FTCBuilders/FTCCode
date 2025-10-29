package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class OuttakeMotor {

    private DcMotor outtakeMotor;

    public OuttakeMotor(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Run motor at given power
    public void start(double power){
        outtakeMotor.setPower(power);
    }

    // Stop motor
    public void stop(){
        outtakeMotor.setPower(0);
    }

    // Get current motor power
    public double power(){
        return outtakeMotor.getPower();
    }


}
