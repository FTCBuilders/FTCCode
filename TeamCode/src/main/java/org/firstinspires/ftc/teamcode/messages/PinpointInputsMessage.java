package org.firstinspires.ftc.teamcode.messages;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class PinpointInputsMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;
    public double xVel;
    public double yVel;
    public double headingVel;

    public PinpointInputsMessage(Pose2D position, Pose2D velocity) {
        this.timestamp = System.nanoTime();
        this.x = position.getX(DistanceUnit.INCH);
        this.y = position.getY(DistanceUnit.INCH);
        this.heading = position.getHeading(AngleUnit.RADIANS);
        this.xVel = velocity.getX(DistanceUnit.INCH);
        this.yVel = velocity.getY(DistanceUnit.INCH);
        this.headingVel = velocity.getHeading(AngleUnit.RADIANS);
    }
}
