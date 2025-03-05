package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    /** Positions measured in meters and angle in degrees */
    public enum ReefFace {
        AB(Units.inchesToMeters(144), Units.inchesToMeters(158), 2, 0, 180),
        CD(Units.inchesToMeters(160), Units.inchesToMeters(130), 2, 0, 240),
        EF(Units.inchesToMeters(193), Units.inchesToMeters(130), 2, 0, 300),
        GH(Units.inchesToMeters(209), Units.inchesToMeters(158), 2, 0, 0),
        IJ(Units.inchesToMeters(193), Units.inchesToMeters(186), 2, 0, 60),
        KL(Units.inchesToMeters(160), Units.inchesToMeters(186), 2, 0, 120);

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

        public Pose2d getLeftBranchGoalBlue() { return leftBranchGoal; }
        public Pose2d getRightBranchGoalBlue() { return rightBranchGoal; }
        public Pose2d getLeftBranchGoalRed() { 
            return new Pose2d(
                new Translation2d(18.75 - leftBranchGoal.getX(), leftBranchGoal.getY()), 
                Rotation2d.fromDegrees(leftBranchGoal.getRotation().getDegrees() + 180)
            );
        }
        public Pose2d getRightBranchGoalRed() { 
            return new Pose2d(
                new Translation2d(18.75 - rightBranchGoal.getX(), rightBranchGoal.getY()), 
                Rotation2d.fromDegrees(rightBranchGoal.getRotation().getDegrees() + 180)
            );
        }
    }
}
