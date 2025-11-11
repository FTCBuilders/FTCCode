package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "RRSAuto", group = "Main")
public class RRSAuto extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map motors to their config names
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        // Reverse the right side so forward power makes the robot go forward
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        // Make sure motors are ready to run



        telemetry.addLine("Ready — Press START");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward
            double power = 0.5;
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            sleep(3000); // move forward for 3 seconds

            // Stop motors
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            telemetry.addLine("Done!");
            telemetry.update();
        }
    }
}
