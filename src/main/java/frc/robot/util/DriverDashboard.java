package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriverDashboard {
    
    public DriverDashboard(RobotContext robotContext) {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver");
        HttpCamera httpCamera = new HttpCamera("Limelight Camera", "http://frcvision.local:1181/stream.mjpg"); //TODO fix
        CameraServer.addCamera(httpCamera);
        tab.add(httpCamera).withPosition(0, 0).withSize(5, 5);
    }


}
