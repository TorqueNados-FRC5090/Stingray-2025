package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
    /** Positions measured in meters and angle in degrees */
    public enum ReefFace {
        AB(3.235, 4.248, 2, 0, 0),
        CD(999, 999, 2, 0, 60),
        EF(999, 999, 2, 0, 120),
        GH(5.73, 3.76, 2, 0, 180),
        IJ(999, 999, 2, 0, 240),
        KL(999, 999, 2, 0, 300);

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
