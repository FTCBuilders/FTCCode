package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier leftY, leftX, rightX;

    public TeleopDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drivetrain = drivetrain;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(
                leftY.getAsDouble(),
                leftX.getAsDouble(),
                rightX.getAsDouble()
        );
    }
}