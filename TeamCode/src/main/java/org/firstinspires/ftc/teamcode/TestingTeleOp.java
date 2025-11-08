package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestingTeleOp", group = "Linear OpMode")
public class TestingTeleOp extends LinearOpMode {

    private CustomMecanumDrive mecanumDrive;
    private intakeMotor intakeMotor;
    private transferMotor transferMotor;
    private OuttakeMotor outtakeMotor;

    // ----- TOGGLE STATES -----
    private boolean intakeOn = false;
    private boolean outtakeOn = false;

    // ----- BUTTON STATE TRACKING -----
    private boolean lastIntakeButton = false;
    private boolean lastOuttakeButton = false;

    private long shootStartTime = 0;

    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
        intakeMotor = new intakeMotor(hardwareMap);
        transferMotor = new transferMotor(hardwareMap);
        outtakeMotor = new OuttakeMotor(hardwareMap);

        telemetry.addLine("Initialized — Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----- DRIVE CONTROL -----
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
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

            // ----- SHOOT CONTROL (right trigger) -----
            double flywheelSpeed = 0.75;
            boolean shootPressed = gamepad1.right_trigger > 0;  // adjust threshold
            boolean isShooting = false;

            if (shootPressed) {
                isShooting = !isShooting;
            }

            if (isShooting) {
                // Spin up flywheel
                outtakeMotor.start(-flywheelSpeed);

                // Once at speed (or after small delay), feed ball
                // We'll use a simple timer-based delay here
                long currentTime = System.currentTimeMillis();
                if (shootStartTime == 0) shootStartTime = currentTime; // mark the first press
                if (currentTime - shootStartTime > 2500) {  // wait 2.5s for spin-up
                    intakeMotor.setPower(-1.0);      // run intake forward
                    transferMotor.setPower(-1.0);    // run transfer forward
                }

            } else {
                // Stop everything when released
                shootStartTime = 0;
                outtakeMotor.stop();
                intakeMotor.stop();
                transferMotor.stop();
            }

            // ----- TELEMETRY -----
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("Reverse Mode", reverseMode ? "ON (Left Trigger)" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Transfer", gamepad1.right_trigger > 0 ? "ON" : "OFF");
            telemetry.addData("Outtake", outtakeOn ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
