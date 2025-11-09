package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OfficialTeleOpTwoDrivers", group = "Linear OpMode")
public class OfficialTeleOpTwoDrivers extends LinearOpMode {

    private CustomMecanumDrive mecanumDrive;
    private intakeMotor intakeMotor;
    private transferMotor transferMotor;
    private PIDOuttakeMotor outtakeMotor;

    // ----- TOGGLE STATES -----
    private boolean intakeOn = false;
    private boolean outtakeOn = false;

    // ----- BUTTON STATE TRACKING -----
    private boolean lastIntakeButton = false;
    private boolean lastOuttakeButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private double applyDeadband(double value, double threshold) {
        return Math.abs(value) > threshold ? value : 0.0;
    }

    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
        intakeMotor = new intakeMotor(hardwareMap);
        transferMotor = new transferMotor(hardwareMap);
        outtakeMotor = new PIDOuttakeMotor(hardwareMap);
        int targetTicksPerSecond = 1500;

        telemetry.addLine("Initialized — Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----- DRIVE CONTROL -----
            double forward = -applyDeadband(gamepad1.left_stick_y,0.05);
            double strafe = applyDeadband(gamepad1.left_stick_x,0.05);
            double rotate = applyDeadband(gamepad1.right_stick_x,0.05);

            if (Math.abs(forward) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(rotate) < 0.05)
                mecanumDrive.stop();
            else
                mecanumDrive.setDrivePower(forward, strafe, rotate);

            mecanumDrive.setDrivePower(forward, strafe, rotate);

            // Check if left trigger is pressed (reverse mode)
            boolean reverseMode = gamepad2.left_trigger > 0.1;
            double direction = reverseMode ? 1.0 : -1.0;

            // ----- INTAKE TOGGLE (A button) -----
            boolean currentIntakeButton = gamepad2.a;
            if (currentIntakeButton && !lastIntakeButton) {
                intakeOn = !intakeOn;
            }
            lastIntakeButton = currentIntakeButton;

            if (intakeOn) intakeMotor.setPower(direction);
            else intakeMotor.stop();

            // ----- TRANSFER (right trigger) -----
            boolean transferPressed = gamepad2.right_trigger > 0.1;
            if (transferPressed) transferMotor.setPower(direction);
            else transferMotor.stop();

            // ----- DPAD UP/DOWN (adjust flywheel speed once per press) -----
            boolean currentDpadUp = gamepad2.dpad_up;
            boolean currentDpadDown = gamepad2.dpad_down;

            if (currentDpadUp && !lastDpadUp) {
                targetTicksPerSecond += 50;
            }
            if (currentDpadDown && !lastDpadDown) {
                targetTicksPerSecond -= 50;
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // ----- OUTTAKE TOGGLE (right bumper) -----
            boolean currentOuttakeButton = gamepad2.right_bumper;
            if (currentOuttakeButton && !lastOuttakeButton) {
                outtakeOn = !outtakeOn;
            }
            lastOuttakeButton = currentOuttakeButton;

            if (outtakeOn) {
                outtakeMotor.start(targetTicksPerSecond);
            } else {
                outtakeMotor.stop();
            }

            outtakeMotor.update();

            // ----- TELEMETRY -----
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("Reverse mode", reverseMode ? "ON (Left Trigger)" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Transfer", transferPressed ? "ON" : "OFF");
            telemetry.addData("Flywheel Target Ticks Per Second", targetTicksPerSecond);
            telemetry.addData("Flywheel Target", outtakeMotor.getTargetVelocity());
            telemetry.addData("Flywheel Actual", outtakeMotor.getVelocity());
            telemetry.addData("Flywheel Error", outtakeMotor.getTargetVelocity() - outtakeMotor.getVelocity());
            telemetry.update();
        }
    }
}
