package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
    /** Positions measured in meters and angle in degrees */
    public enum ReefFace {
        AB(0, 0, 0, 0, 180),
        CD(0, 0, 0, 0, 240),
        EF(0, 0, 0, 0, 300),
        GH(0, 0, 0, 0, 0),
        IJ(0, 0, 0, 0, 60),
        KL(0, 0, 0, 0, 120);

        private Pose2d leftBranchGoal;
        private Pose2d rightBranchGoal;

        ReefFace(double leftX, double leftY, double rightX, double rightY, double angle) {
            this.leftBranchGoal = new Pose2d(
                new Translation2d(leftX, leftY), 
                Rotation2d.fromDegrees(angle)
            );

            this.rightBranchGoal = new Pose2d(
                new Translation2d(rightX, rightY), 
                Rotation2d.fromDegrees(angle)
            );
        }

        public Pose2d getLeftBranchGoal() { return leftBranchGoal; }
        public Pose2d getRightBranchGoal() { return rightBranchGoal; }
    }
}
