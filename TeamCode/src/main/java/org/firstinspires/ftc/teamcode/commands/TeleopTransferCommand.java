package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.function.BooleanSupplier;

public class TeleopTransferCommand extends CommandBase {
    private final TransferSubsystem transfer;
    private final BooleanSupplier buttonValue;

    public TeleopTransferCommand(TransferSubsystem transfer, BooleanSupplier buttonValue) {
        this.transfer = transfer;
        this.buttonValue = buttonValue;
        addRequirements(transfer);
    }

    @Override
    public void execute() {
        transfer.updateButton(buttonValue.getAsBoolean());
    }
}
