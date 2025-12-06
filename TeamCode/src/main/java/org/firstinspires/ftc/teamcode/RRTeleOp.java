package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RRTeleOp", group = "Linear OpMode")
public class RRTeleOp extends LinearOpMode {

    private CustomMecanumDrive mecanumDrive;
    private OuttakeMotor outtakeMotor;
    private intakeMotor intakeMotor;
    private transferMotor transferMotor;

    // ----- Intake -----
    private boolean intakeRunning = false;
    private boolean lbPressedLast = false;

    // ----- Outtake -----
    private boolean outtakeRunning = false;
    private boolean rbPressedLast = false;

    // ----- Transfer -----
    private boolean transferRunning = false;
    private boolean transferReversed = false;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

    // ----- Flywheel velocity control -----
    private double targetVelocity = 1200; // initial ticks/sec
    private static final double VELOCITY_INCREMENT = 50;
    private static final double MIN_VELOCITY = 0;
    private static final double MAX_VELOCITY = 1880;
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;


    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
        outtakeMotor = new OuttakeMotor(hardwareMap);
        intakeMotor = new intakeMotor(hardwareMap);
        transferMotor = new transferMotor(hardwareMap);

        telemetry.addLine("Initialized — Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----- DRIVE CONTROL -----
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            mecanumDrive.setDrivePower(forward, strafe, rotate);


            // ----- TRANSFER CONTROL (NON-TOGGLE) -----
            double transferPower = 0;

            // Hold A → forward
            if (gamepad2.a) {
                transferPower = 1.0;
            }
            // Hold B → reverse
            else if (gamepad2.b) {
                transferPower = -1.0;
            }
            // Neither pressed → stop
            else {
                transferPower = 0;
            }
            transferMotor.setPower(transferPower);


            // ----- INTAKE CONTROL -----
            if (gamepad2.left_bumper && !lbPressedLast) {
                intakeRunning = !intakeRunning;
            }
            lbPressedLast = gamepad2.left_bumper;

            double intakePower = 0;
            if (intakeRunning) {
                intakePower = (gamepad2.left_trigger > 0.1) ? 1.0 : -1.0; // reverse while LT held
            }
            intakeMotor.setPower(intakePower);

            // ----- OUTTAKE CONTROL -----
            if (gamepad2.right_bumper && !rbPressedLast) {
                outtakeRunning = !outtakeRunning;
            }
            rbPressedLast = gamepad2.right_bumper;

            // ----- D-PAD TO ADJUST VELOCITY -----
            if (gamepad2.dpad_up && !dpadUpLast) {
                targetVelocity += VELOCITY_INCREMENT;
                if (targetVelocity > MAX_VELOCITY) targetVelocity = MAX_VELOCITY;
            }
            if (gamepad2.dpad_down && !dpadDownLast) {
                targetVelocity -= VELOCITY_INCREMENT;
                if (targetVelocity < MIN_VELOCITY) targetVelocity = MIN_VELOCITY;
            }
            dpadUpLast = gamepad2.dpad_up;
            dpadDownLast = gamepad2.dpad_down;

            // Start/stop the flywheel using PIDF
            if (outtakeRunning) {
                outtakeMotor.start(targetVelocity);
            } else {
                outtakeMotor.stop();
            }
            outtakeMotor.update();

            // ----- TELEMETRY -----
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Transfer Power", transferMotor.getPower());
            telemetry.addData("Transfer Reversed", transferReversed);
            telemetry.addData("Outtake Running", outtakeRunning);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", outtakeMotor.getCurrentVelocity());
            telemetry.update();
        }
    }
}
