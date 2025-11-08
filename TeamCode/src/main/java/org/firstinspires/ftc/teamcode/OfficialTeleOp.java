package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OfficialTeleOp", group = "Linear OpMode")
public class OfficialTeleOp extends LinearOpMode {

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

    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
        intakeMotor = new intakeMotor(hardwareMap);
        transferMotor = new transferMotor(hardwareMap);
        outtakeMotor = new PIDOuttakeMotor(hardwareMap);

        telemetry.addLine("Initialized — Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----- DRIVE CONTROL -----
            double forward = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double rotate = gamepad2.right_stick_x;
            mecanumDrive.setDrivePower(forward, strafe, rotate);

            // Check if left trigger is pressed (reverse mode)
            boolean reverseMode = gamepad1.left_trigger > 0; // Adjust threshold as needed
            double direction = reverseMode ? 1.0 : -1.0; // flip motor directions

            // ----- INTAKE TOGGLE (A button) -----
            if (gamepad1.a && !lastIntakeButton) {
                intakeOn = !intakeOn;
            }
            lastIntakeButton = gamepad1.a;

            if (intakeOn) intakeMotor.setPower(direction);
            else intakeMotor.stop();

            // ----- TRANSFER (right trigger) -----
            if (gamepad1.right_trigger > 0) {
                transferMotor.setPower(direction);
            } else {
                transferMotor.stop();
            }

            // ----- OUTTAKE TOGGLE (right bumper) -----
            if (gamepad1.right_bumper && !lastOuttakeButton) {
                outtakeOn = !outtakeOn;
            }
            lastOuttakeButton = gamepad1.right_bumper;

            if (outtakeOn) {
                double targetVelocity = 29000;  // ≈4500 RPM
                outtakeMotor.start(targetVelocity);
            } else {
                outtakeMotor.stop();
            }

            // ----- TELEMETRY -----
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("Reverse mode", reverseMode ? "ON (Left Trigger)" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Transfer", gamepad1.right_trigger > 0 ? "ON" : "OFF");
            telemetry.addData("Flywheel Target", outtakeMotor.getTargetVelocity());
            telemetry.addData("Flywheel Actual", outtakeMotor.getVelocity());
            telemetry.addData("Flywheel Error",
                    outtakeMotor.getTargetVelocity() - outtakeMotor.getVelocity());
            telemetry.update();
        }
    }
}
