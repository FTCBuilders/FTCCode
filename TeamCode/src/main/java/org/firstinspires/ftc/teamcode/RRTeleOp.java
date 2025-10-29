package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RRTeleOp", group = "Linear OpMode")
public class RRTeleOp extends LinearOpMode {

    private CustomMecanumDrive mecanumDrive; // your custom drive
    private OuttakeMotor intakeMotor;
    private OuttakeMotor outtake;
    private transferMotor transferMotor;

    // For toggle detection
    boolean intakeForward = false;
    boolean intakeBackward = false;
    private boolean transferForward = false;
    private boolean transferBackward = false;
    private boolean yPressedLast = false;


    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
        outtake = new OuttakeMotor(hardwareMap);
        intakeMotor = outtake;
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

            // ----- TRANSFER SUBSYSTEM CONTROL -----
            if (gamepad1.y && !yPressedLast) {
                if (transferBackward) {
                    transferBackward = false;  // turn off backward
                } else {
                    transferBackward = true;   // start backward
                    transferForward = false;   // ensure forward is off
                }
            }
            yPressedLast = gamepad1.y;  // update last state

// Set motor power based on toggle state
            if (transferForward) {
                transferMotor.setPower(1.0);   // forward
            } else if (transferBackward) {
                transferMotor.setPower(-1.0);  // backward
            } else {
                transferMotor.stop();           // stopped
            }

            // Backward toggle


            // ----- INTAKE TOGGLE CONTROL -----
            // Forward toggle
            if (gamepad1.dpad_up) {
                if (intakeForward) {
                    intakeForward = false;
                } else {
                    intakeForward = true;
                    intakeBackward = false;
                }
            }

            // Backward toggle
            if (gamepad1.dpad_down) {
                if (intakeBackward) {
                    intakeBackward = false;
                } else {
                    intakeBackward = true;
                    intakeForward = false;
                }
            }

            if (intakeForward) {
                intakeMotor.start(1.0);
            } else if (intakeBackward) {
                intakeMotor.start(-1.0);
            } else {
                intakeMotor.stop();
            }


            // ----- OUTTAKE CONTROL -----
            if (gamepad1.a) outtake.start(1.0);
            else outtake.stop();

            // ----- TELEMETRY -----
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("TransferMotor Power", transferMotor.getPower());
            telemetry.addData("IntakeMotor Power", intakeMotor.power());
            telemetry.addData("Outtake Power", outtake.power());
            telemetry.update();
        }
    }
}