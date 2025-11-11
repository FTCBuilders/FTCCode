package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.TimeTrajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "RR Auto", group = "Main")
public class RRAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // intake outtake motors
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        DcMotor outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");

        waitForStart();
        if (isStopRequested()) return;

        // start 45 degrees
        Action initialTurn = drive.actionBuilder(startPose)
                .turn(Math.toRadians(45))
                .build();
        Actions.runBlocking(initialTurn);

        // Get current pose after 45° turn
        Pose2d poseAfterTurn = drive.localizer.getPose();

        // Distance to move along heading
        double distanceInches = 2 * 39.3701; // 2 meters

        // Convert heading to double radians
        double headingRad = poseAfterTurn.heading.toDouble(); // <-- rotation in radians

        // Compute target pose along current heading
        double targetX = poseAfterTurn.position.x + distanceInches * Math.cos(headingRad);
        double targetY = poseAfterTurn.position.y + distanceInches * Math.sin(headingRad);


        Pose2d targetPose = new Pose2d(targetX, targetY, headingRad);

        // Use the correct method your Roadrunner version accepts
        Action diagonalMove = drive.actionBuilder(poseAfterTurn)
                .strafeToConstantHeading(targetPose.position)  // OR splineToSplineHeading(targetPose)
                .build();
        Actions.runBlocking(diagonalMove);

        //shoots 2 balls
        // 3️⃣ Shoot balls
        outtakeMotor.setPower(1);      // shooter
        transferMotor.setPower(1);     // feed balls
        intakeMotor.setPower(0.5);     // optional: help feed balls
        sleep(2000);                   // shoot for 1.5 seconds

        // Stop all motors
        outtakeMotor.setPower(0);
        transferMotor.setPower(0);
        intakeMotor.setPower(0);

        // Telemetry: show final pose
        Pose2d finalPose = drive.localizer.getPose();
        telemetry.addData("X (in)", finalPose.position.x);
        telemetry.addData("Y (in)", finalPose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.update();

        sleep(1000);
    }
}