// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // STEP 2a: put your CAN IDs here
  private final TalonFX motorB = new TalonFX(15);

  // Reuse one control object (clean + efficient)
  private final DutyCycleOut duty = new DutyCycleOut(0);

  public IntakeSubsystem() {
   
  }

  // STEP 2c: Run intake at a percent output (-1.0 to 1.0)
  public void run(double percent) {
    motorB.setControl(duty.withOutput(percent));
  }

  public void stop() {
    run(0.0);
  }
}
