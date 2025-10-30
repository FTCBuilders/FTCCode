package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RRTeleOp", group = "Linear OpMode")
public class RRTeleOp extends LinearOpMode {

    private CustomMecanumDrive mecanumDrive; // your custom drive
    private OuttakeMotor intakeOuttakeMotor; // single motor for both intake and outtake
    private transferMotor transferMotor;


    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
        intakeOuttakeMotor = new OuttakeMotor(hardwareMap);
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
            // Hold Y for forward, hold X for backward
            if (gamepad1.y) {
                transferMotor.setPower(1.0);   // forward
            } else if (gamepad1.x) {
                transferMotor.setPower(-1.0);  // backward
            } else {
                transferMotor.stop();          // stopped
            }

            // ----- INTAKE/OUTTAKE CONTROL -----
            // Hold dpad_up for intake, hold dpad_down for outtake
            if (gamepad1.dpad_up) {
                intakeOuttakeMotor.start(1.0);  // intake
            } else if (gamepad1.dpad_down) {
                intakeOuttakeMotor.start(-1.0); // outtake
            } else {
                intakeOuttakeMotor.stop();      // stopped
            }

            // ----- TELEMETRY -----
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("TransferMotor Power", transferMotor.getPower());
            telemetry.addData("Intake/Outtake Power", intakeOuttakeMotor.power());
            telemetry.update();
        }
    }
}