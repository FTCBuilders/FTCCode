package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TransferSubsystem extends SubsystemBase {
    private final Motor transfer;
    private Telemetry telemetry;

    private boolean transferOn = false;
    private boolean lastButtonState = false;

    public TransferSubsystem(HardwareMap hardwareMap) {
        transfer = new Motor(hardwareMap, "transferMotor");
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void updateButton(boolean currentButtonState) {
        if (currentButtonState && !lastButtonState) {
            transferOn = !transferOn;
            transfer.set(transferOn ? 1 : 0);
        }

        lastButtonState = currentButtonState;
    }
    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.addData("Transfer On?", transferOn);
            telemetry.addData("Transfer Velocity (ticks/s)", transfer.getCorrectedVelocity());
            telemetry.update();
        }
    }
}
