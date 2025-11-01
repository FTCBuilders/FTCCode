package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RRSAuto", group = "Main")
public class RRSAuto extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        // Reverse left side if needed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized — Waiting for start");
        telemetry.update();

        // Wait for start button
        waitForStart();

        if (opModeIsActive()) {
            // Move forward for 5 seconds
            double power = 0.5; // 50% power
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            sleep(1000); // Move for 5 seconds (5000 milliseconds)

            // Stop all motors
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            telemetry.addLine("Autonomous complete!");
            telemetry.update();
        }
    }
}