package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriverDashboard {
  private VideoCamera webCam;

  public DriverDashboard(RobotContext robotContext) {
    ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    webCam = CameraServer.startAutomaticCapture();
    webCam.setVideoMode(PixelFormat.kMJPEG, 200, 200, 20);
    tab.add(webCam).withPosition(5, 1).withSize(3, 3);
  }

  private VideoCamera getLimelight() {
    // HttpCamera httpCamera = new HttpCamera("Limelight Camera", \
    // "http://frcvision.local:1181/stream.mjpg"); //TODO fix
    return null;
  }
}
