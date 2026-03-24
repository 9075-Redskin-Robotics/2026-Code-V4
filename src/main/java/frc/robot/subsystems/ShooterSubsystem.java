// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // STEP 2a: put your CAN IDs here
  private final TalonFX motorA = new TalonFX(14);;

  private static final double kMinAutoShooterOutput = 0.45;
  private static final double kMaxAutoShooterOutput = 1.0;
  private static final double kBaseDistanceMeters = 0.75;
  private static final double kBaseShooterOutput = 0.52;
  private static final double kShooterOutputPerMeter = 0.12;

  // Reuse one control object (clean + efficient)
  private final DutyCycleOut duty = new DutyCycleOut(0);

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Output Command", duty.Output);
  }

  // STEP 2c: Run intake at a percent output (-1.0 to 1.0)
  public void run(double percent) {
    motorA.setControl(duty.withOutput(percent));
  }

  public void runForDistance(double distanceMeters) {
    run(getOutputForDistance(distanceMeters));
  }

  public double getOutputForDistance(double distanceMeters) {
    double shooterOutput = kBaseShooterOutput
        + ((distanceMeters - kBaseDistanceMeters) * kShooterOutputPerMeter);
    return MathUtil.clamp(shooterOutput, kMinAutoShooterOutput, kMaxAutoShooterOutput);
  }

  public void stop() {
    run(0.0);
  }
}
