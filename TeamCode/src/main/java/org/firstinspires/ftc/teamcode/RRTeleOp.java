package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "RRTeleOp", group = "Linear OpMode")
public class RRTeleOp extends LinearOpMode {

    private CustomMecanumDrive mecanumDrive;
    private OuttakeMotor outtakeMotor;
    private intakeMotor intakeMotor;
    private transferMotor transferMotor;


    // Intake
    private boolean intakeRunning = false;
    private boolean lbPressedLast = false;
    private boolean intakeReversed = false;

    // Outtake
    private boolean outtakeRunning = false;
    private boolean rbPressedLast = false;


    // Transfer toggles
    private boolean transferRunning = false;
    private boolean transferReversed = false;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

    @Override
    public void runOpMode() {

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

            // ----- TRANSFER CONTROL -----
            if (gamepad2.a && !aPressedLast) {
                transferRunning = !transferRunning;
                transferReversed = false;
            }
            aPressedLast = gamepad2.a;

            if (gamepad2.b && !bPressedLast) {
                transferRunning = !transferRunning;
                transferReversed = true;
            }
            bPressedLast = gamepad2.b;

            if (transferRunning) {
                double power = transferReversed ? -0.8 : 0.8;
                transferMotor.setPower(power);
            } else {
                transferMotor.setPower(0);
            }

            // ----- INTAKE / OUTTAKE CONTROL -----

            // ----- INTAKE CONTROL -----
            if (gamepad2.left_bumper && !lbPressedLast) {
                intakeRunning = !intakeRunning;   // toggle intake on/off
            }
            lbPressedLast = gamepad2.left_bumper;

            double intakePower = 0;
            if (intakeRunning) {
                intakePower = (gamepad2.left_trigger > 0.1) ? -1.0 : 1.0;  // reverse while LT held
            }
            intakeMotor.setPower(intakePower);

            // ----- OUTTAKE CONTROL -----
            if (gamepad2.right_bumper && !rbPressedLast) {
                outtakeRunning = !outtakeRunning; // toggle outtake on/off
            }
            rbPressedLast = gamepad2.right_bumper;

            double targetVelocity = 0;
            if (outtakeRunning) {
                double targetPower = 0.67; // same as before
                targetVelocity = targetPower * 500;
                outtakeMotor.setVelocity(targetVelocity);
            } else {
                outtakeMotor.stop();
            }
            // ----- OUTTAKE TELEMETRY -----
            telemetry.addData("Outtake Status", outtakeRunning ? "PID Active" : "Stopped");
            telemetry.addData("Outtake Target Vel", outtakeRunning ? targetVelocity : 0);
            telemetry.addData("Outtake Current Vel", outtakeMotor.getCurrentVelocity());


            // ----- TELEMETRY -----
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("Transfer Power", transferMotor.getPower());
            telemetry.addData("Transfer Reversed", transferReversed);
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Outtake Power", outtakeMotor.getCurrentVelocity());
            telemetry.addData("Intake Reversed", intakeReversed);
            telemetry.update();
        }
    }
}
