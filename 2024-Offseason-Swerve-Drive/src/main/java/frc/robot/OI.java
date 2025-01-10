// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI extends CommandXboxController {

    public OI(int port) {
        super(port);
    }

    static OI oi = null;

    public static OI getInstance() {
        if (oi == null) {
            oi = new OI(0);
        }
        return oi;
    }
}
