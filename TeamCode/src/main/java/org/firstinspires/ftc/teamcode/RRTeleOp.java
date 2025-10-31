package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RRTeleOp", group = "Linear OpMode")
public class RRTeleOp extends LinearOpMode {

    private CustomMecanumDrive mecanumDrive; // your custom drive
    private OuttakeMotor outtakeMotor;

    private intakeMotor intakeMotor;// single motor for both intake and outtake
    private transferMotor transferMotor;


    private boolean intakeOuttakeRunning = false; // toggle state
    private boolean aPressedLast = false;
    private boolean transferRunning = false;   // current toggle state
    private boolean yPressedLast = false;      // track Y button press



    @Override
    public void runOpMode() {

        // Initialize subsystems
        mecanumDrive = new CustomMecanumDrive(hardwareMap);
         outtakeMotor =  new OuttakeMotor(hardwareMap);
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

            // ----- TRANSFER SUBSYSTEM CONTROL -----
            // Hold Y for forward, hold X for backward
            if (gamepad1.y && !yPressedLast) {       // detect new press
                transferRunning = !transferRunning;  // flip the toggle
            }
            yPressedLast = gamepad1.y;

            if (transferRunning) {
                transferMotor.setPower(.8);        // run forward
            } else {
                transferMotor.stop();                // stop
            }

            // ----- INTAKE/OUTTAKE CONTROL -----
            // Hold dpad_up for intake, hold dpad_down for outtake
            if (gamepad1.a && !aPressedLast) {
                intakeOuttakeRunning = !intakeOuttakeRunning; // toggle state
            }
            aPressedLast = gamepad1.a;

            if (intakeOuttakeRunning) {
                intakeMotor.setPower(.8);   // run intake
                outtakeMotor.setPower(.8);  // run outtake at same time
            } else {
                intakeMotor.stop();
                outtakeMotor.stop();
            }



            // ----- TELEMETRY -----
            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, rotate);
            telemetry.addData("TransferMotor Power", transferMotor.getPower());
            telemetry.addData("intakeOuttakeMotor", intakeMotor.getPower());
            telemetry.update();
        }
    }
}