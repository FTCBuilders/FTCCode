package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopFlywheelCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Main Teleop Program")
public class MainTeleop extends CommandOpMode {

    private DrivetrainSubsystem drivetrain;
    private FlywheelSubsystem flywheel;
    private TransferSubsystem transfer;
    private IntakeSubsystem intake;
    private GamepadEx driverController;

    @Override
    public void initialize() {
        Pose2d initialPose = new Pose2d(0, 0, 0);

        drivetrain = new DrivetrainSubsystem(hardwareMap, initialPose);
        drivetrain.setTelemetry(telemetry);
        driverController = new GamepadEx(gamepad1);

        flywheel = new FlywheelSubsystem(hardwareMap);
        flywheel.setTelemetry(telemetry);

        transfer = new TransferSubsystem(hardwareMap);
        transfer.setTelemetry(telemetry);

        intake = new IntakeSubsystem(hardwareMap);
        intake.setTelemetry(telemetry);

        drivetrain.setDefaultCommand(
                new TeleopDriveCommand(
                        drivetrain,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> driverController.getRightX()
                )
        );

        flywheel.setDefaultCommand(
                new TeleopFlywheelCommand(
                        flywheel,
                        () -> driverController.getButton(GamepadKeys.Button.Y)
                )
        );

        transfer.setDefaultCommand(
                new TeleopTransferCommand(
                        transfer,
                        () -> driverController.getButton(GamepadKeys.Button.X)
                )
        );

        intake.setDefaultCommand(
                new TeleopIntakeCommand(
                        intake,
                        () -> driverController.getButton(GamepadKeys.Button.A)
                )
        );
    }
}