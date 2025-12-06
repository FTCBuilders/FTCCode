package com.example.decodeactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public class DecodeActions {
    public static final double ROBOT_WIDTH = 18;
    public static final double ROBOT_HEIGHT = 18;

    public Action getInitialAction(TrajectoryActionBuilder trajectoryActionBuilder) {

        return trajectoryActionBuilder
                // 1. Small forward into shooting position
                .lineToX(-4)

                // 2. Rotate to shooting angle (clockwise / counter-clockwise)
                .turn(Math.toRadians(53))

                // --- Shooting would happen here ---

                // 3. Turn right toward human player area
                .turn(Math.toRadians(-145))

                // 4. Move forward slightly
                .lineToX(0.01)

                // 5. Move back into original position
                .lineToX(-4)

                // 6. Rotate back to shooting angle
                .turn(Math.toRadians(145))

                // 7. Minor adjustments (if needed)
                .turn(Math.toRadians(-145))
                .lineToX(0.01)
                .lineToX(-4)
                .turn(Math.toRadians(145))

                .build();
    }
}
