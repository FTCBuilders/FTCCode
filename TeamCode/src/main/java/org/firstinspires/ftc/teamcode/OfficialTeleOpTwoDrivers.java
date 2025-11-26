package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "OfficialTeleOpTwoDrivers", group = "Linear OpMode")
public class OfficialTeleOpTwoDrivers extends LinearOpMode {

    private CustomMecanumDrive mecanumDrive;
    private intakeMotor intakeMotor;
    private transferMotor transferMotor;
    private PIDOuttakeMotors outtakeMotors;

    // ----- TOGGLE STATES -----
    private boolean intakeOn = false;
    private boolean outtakeOn = false;

    // ----- BUTTON STATE TRACKING -----
    private boolean lastIntakeButton = false;
    private boolean lastOuttakeButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    Gamepad driveController;
    Gamepad ballController;

    private double applyDeadband(double value, double threshold) {
        return Math.abs(value) > threshold ? value : 0.0;
    }

    private void setupControllers() {
        // Run with a single controller if the other one has not been seen yet
        if (gamepad1.getGamepadId() > -1 && gamepad2.getGamepadId() == -1) {
            // gamepad2 seems disconnected, so let's use gamepad1 for everything
            driveController = gamepad1;
            ballController = gamepad1;
        } else if (gamepad1.getGamepadId() == -1 && gamepad2.getGamepadId() > -1) {
            // gamepad1 seems disconnected, so let's use gamepad2 for everything
            driveController = gamepad2;
            ballController = gamepad2;
        } else {
            // Both gamepads are in the same state, so assign them to their default roles
            driveController = gamepad1;
            ballController = gamepad2;
        }
    }

    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
        intakeMotor = new intakeMotor(hardwareMap);
        transferMotor = new transferMotor(hardwareMap);
        outtakeMotors = new PIDOuttakeMotors(hardwareMap, "outtakeMotor", "outtakeMotor2");
        int targetTicksPerSecond = 1500;

        telemetry.addLine("Initialized — Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            setupControllers();

            // ----- DRIVE CONTROL -----
            double forward = -applyDeadband(driveController.left_stick_y,0.05);
            double strafe = applyDeadband(driveController.left_stick_x,0.05);
            double rotate = applyDeadband(driveController.right_stick_x,0.05);

            if (Math.abs(forward) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(rotate) < 0.05)
                mecanumDrive.stop();
            else
                mecanumDrive.setDrivePower(forward, strafe, rotate);

            mecanumDrive.setDrivePower(forward, strafe, rotate);

            // Check if left trigger is pressed (reverse mode)
            boolean reverseMode = ballController.left_trigger > 0.1;
            double direction = reverseMode ? 1.0 : -1.0;

            // ----- INTAKE TOGGLE (A button) -----
            boolean currentIntakeButton = ballController.a;
            if (currentIntakeButton && !lastIntakeButton) {
                intakeOn = !intakeOn;
            }
            lastIntakeButton = currentIntakeButton;

            if (intakeOn) intakeMotor.setPower(direction);
            else intakeMotor.stop();

            // ----- TRANSFER (right trigger) -----
            boolean transferPressed = ballController.right_trigger > 0.1;
            if (transferPressed) transferMotor.setPower(direction);
            else transferMotor.stop();

            // ----- DPAD UP/DOWN (adjust flywheel speed once per press) -----
            boolean currentDpadUp = ballController.dpad_up;
            boolean currentDpadDown = ballController.dpad_down;

            if (currentDpadUp && !lastDpadUp) {
                targetTicksPerSecond += 50;
            }
            if (currentDpadDown && !lastDpadDown) {
                targetTicksPerSecond -= 50;
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // ----- OUTTAKE TOGGLE (right bumper) -----
            boolean currentOuttakeButton = ballController.right_bumper;
            if (currentOuttakeButton && !lastOuttakeButton) {
                outtakeOn = !outtakeOn;
            }
            lastOuttakeButton = currentOuttakeButton;

            if (outtakeOn) {
                outtakeMotors.start(targetTicksPerSecond);
            } else {
                outtakeMotors.stop();
            }

            outtakeMotors.update();

            // ----- TELEMETRY -----
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Transfer", transferPressed ? "ON" : "OFF");
            telemetry.addData("Flywheel", outtakeOn ? "ON" : "OFF");
            telemetry.addData("Flywheel Target Ticks Per Second", targetTicksPerSecond);
            telemetry.addData("Flywheel Actual", outtakeMotors.getAverageVelocity());
            telemetry.addLine("---------------");
            telemetry.addData("Flywheel Error", outtakeMotors.getTargetVelocity() - outtakeMotors.getAverageVelocity());
            telemetry.addData("Reverse mode", reverseMode ? "ON" : "OFF");
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("Drive Controller", driveController == gamepad1 ? "User 1 (BLUE)" : "User 2 (RED)");
            telemetry.addData("Ball Controller", ballController == gamepad1 ? "User 1 (BLUE)" : "User 2 (RED)");
            telemetry.addData("Gamepad1 ID", gamepad1.getGamepadId());
            telemetry.addData("Gamepad2 ID", gamepad2.getGamepadId());

            telemetry.update();
        }
    }
}
