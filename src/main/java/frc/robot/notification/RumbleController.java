package frc.robot.notification;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class RumbleController extends Command {
    
    private final PS4Controller controller;

    public RumbleController(PS4Controller controller) {
        this.controller = controller;
    }

    @Override
    public void initialize() {
        controller.setRumble(RumbleType.kBothRumble, Constants.Controller.RUMBLE_POWER);
    }

    @Override
    public void end(boolean interruped) {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
