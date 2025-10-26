package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelSubsystem extends SubsystemBase {
    private final Motor flywheel;
    private Telemetry telemetry;

    private boolean flywheelOn = false;
    private boolean lastButtonState = false;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        flywheel = new Motor(hardwareMap, "outtakeMotor");
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void updateButton(boolean currentButtonState) {
        if (currentButtonState && !lastButtonState) {
            flywheelOn = !flywheelOn;
            flywheel.set(flywheelOn ? 1 : 0);
        }

        lastButtonState = currentButtonState;
    }

    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.addData("Flywheel On?", flywheelOn);
            telemetry.addData("Flywheel Velocity (ticks/s)", flywheel.getCorrectedVelocity());
            telemetry.update();
        }
    }
}
