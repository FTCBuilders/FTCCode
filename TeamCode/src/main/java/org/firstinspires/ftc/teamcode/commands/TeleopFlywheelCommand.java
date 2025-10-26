package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

import java.util.function.BooleanSupplier;

public class TeleopFlywheelCommand extends CommandBase {
    private final FlywheelSubsystem flywheel;
    private final BooleanSupplier buttonValue;

    public TeleopFlywheelCommand(FlywheelSubsystem flywheel, BooleanSupplier buttonValue) {
        this.flywheel = flywheel;
        this.buttonValue = buttonValue;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        flywheel.updateButton(buttonValue.getAsBoolean());
    }
}
