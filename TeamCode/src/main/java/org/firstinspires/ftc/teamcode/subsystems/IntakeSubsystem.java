package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intake;
    private Telemetry telemetry;

    private boolean intakeOn = false;
    private boolean lastButtonState = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = new Motor(hardwareMap, "intakeMotor");
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void updateButton(boolean currentButtonState) {
        if (currentButtonState && !lastButtonState) {
            intakeOn = !intakeOn;
            intake.set(intakeOn ? 1 : 0);
        }

        lastButtonState = currentButtonState;
    }
    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.addData("Intake On?", intakeOn);
            telemetry.addData("Intake Velocity (ticks/s)", intake.getCorrectedVelocity());
            telemetry.update();
        }
    }
}
