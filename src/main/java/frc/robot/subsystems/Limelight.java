package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

  LimelightInputs inputs = new LimelightInputs();
  NetworkTable table;
  String limelightName = "limelight-shooter";

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  @Override
  public void periodic() {
    updateInputs();
    SmartDashboard.putBoolean("Shooter Limelight Has Target", inputs.seesTag);
    SmartDashboard.putNumber("Shooter Limelight Distance Meters", inputs.targetDistanceMeters);
  }

  public void updateInputs() {

PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

inputs.seesTag = LimelightHelpers.getTV(limelightName);

if (estimate != null) {
  try {
    inputs.visibletags = new double[estimate.rawFiducials.length];
    for (int i = 0; i < inputs.visibletags.length; i++) {
      inputs.visibletags[i] = estimate.rawFiducials[i].id;
    }
    inputs.avgTagArea = estimate.avgTagArea;
    inputs.stdDevs = table.getEntry("stddevs").getDoubleArray(new double[12]);
    inputs.pose = estimate.pose;
    inputs.timestamp = estimate.timestampSeconds;

    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
    inputs.targetXRobotMeters = targetPoseRobotSpace.getX();
    inputs.targetYRobotMeters = targetPoseRobotSpace.getY();
    inputs.targetDistanceMeters = Math.hypot(
      inputs.targetXRobotMeters,
      inputs.targetYRobotMeters);

  } catch (Exception e) {
    System.out.println("Error with Limelight Data:" + e.getMessage());
  }

} else {
  inputs.targetXRobotMeters = 0.0;
  inputs.targetYRobotMeters = 0.0;
  inputs.targetDistanceMeters = 0.0;
}

  }

  public void addVisionMeasurement(CommandSwerveDrivetrain drivetrain) {
    drivetrain.addVisionMeasurement(
      inputs.pose,
      inputs.timestamp,
      VecBuilder.fill(inputs.stdDevs[0], inputs.stdDevs[1], inputs.stdDevs[5])
      );
  }

  public boolean hasTarget() {
    return inputs.seesTag;
  }

  public double getTargetDistanceMeters() {
    return inputs.targetDistanceMeters;
  }

  public double getTargetXMeters() {
    return inputs.targetXRobotMeters;
  }

  public double getTargetYMeters() {
    return inputs.targetYRobotMeters;
  }

  protected class LimelightInputs {

    public boolean seesTag;
    public Pose2d pose;
    public double avgTagArea;
    public double[] stdDevs = new double[12];
    public double[] visibletags = new double[2];
    public double timestamp;
    public double targetXRobotMeters;
    public double targetYRobotMeters;
    public double targetDistanceMeters;

  }
}
