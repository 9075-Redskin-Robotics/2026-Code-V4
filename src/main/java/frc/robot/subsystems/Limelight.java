package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  public void updateInputs() {

PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

if (estimate != null) {
  try {
    inputs.visibletags = new double[estimate.rawFiducials.length];
    for (int i = 0; i < inputs.visibletags.length; i++) {
      inputs.visibletags[i] = estimate.rawFiducials[i].id;
    }
    inputs.avgTagArea = estimate.avgTagArea;
    inputs.seesTag = LimelightHelpers.getTV(limelightName);
    inputs.stdDevs = table.getEntry("stddevs").getDoubleArray(new double[12]);
    inputs.pose = estimate.pose;
    inputs.timestamp = estimate.timestampSeconds;

  } catch (Exception e) {
    System.out.println("Error with Limelight Data:" + e.getMessage());
  }

} 

  }

  public void addVisionMeasurement(CommandSwerveDrivetrain drivetrain) {
    drivetrain.addVisionMeasurement(
      inputs.pose,
      inputs.timestamp,
      VecBuilder.fill(inputs.stdDevs[0], inputs.stdDevs[1], inputs.stdDevs[5])
      );
  }

  protected class LimelightInputs {

    public boolean seesTag;
    public Pose2d pose;
    public double avgTagArea;
    public double[] stdDevs = new double[12];
    public double[] visibletags = new double[2];
    public double timestamp;

  }
}
