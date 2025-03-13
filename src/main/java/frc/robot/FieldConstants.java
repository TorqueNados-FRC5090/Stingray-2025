package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
    /** Positions measured in meters and angle in degrees */
    public enum ReefFace {
        A(3.251, 3.815, 3.240, 4.245, 0),
        B(4.32, 3.074, 4.024, 2.914, 60),
        C(4.944, 2.841, 5.314, 3.127, 120),
        D(5.888, 3.527, 5.737, 4.220, 180),
        E(5.696, 4.844, 5.017, 5.155, 240),
        F(4.089, 5.207, 3.759, 5.002, 300);

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

        public Pose2d getLeftBranchGoalBlue(boolean onRedAlliance) { 
            if (onRedAlliance) {
                return new Pose2d(
                    new Translation2d(17.54 - leftBranchGoal.getX(), leftBranchGoal.getY()), 
                        Rotation2d.fromDegrees((leftBranchGoal.getRotation().getDegrees() + 180) % 360)
                );
            }
            else
                return leftBranchGoal;
        }
        public Pose2d getRightBranchGoalBlue(boolean onRedAlliance) { 
            if (onRedAlliance) {
                return new Pose2d(
                    new Translation2d(17.54 - rightBranchGoal.getX(), rightBranchGoal.getY()), 
                        Rotation2d.fromDegrees((rightBranchGoal.getRotation().getDegrees() + 180) % 360)
                );
            }
            else
                return rightBranchGoal; 
        }
    }
}
