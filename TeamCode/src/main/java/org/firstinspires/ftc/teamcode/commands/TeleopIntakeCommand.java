package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class TeleopIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final BooleanSupplier buttonValue;

    public TeleopIntakeCommand(IntakeSubsystem intake, BooleanSupplier buttonValue) {
        this.intake = intake;
        this.buttonValue = buttonValue;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.updateButton(buttonValue.getAsBoolean());
    }
}
