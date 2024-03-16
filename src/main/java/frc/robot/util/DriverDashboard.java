package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriverDashboard {
  private VideoCamera webCam;

  public DriverDashboard(RobotContext robotContext) {
    ShuffleboardTab tab = Shuffleboard.getTab("Camera");
    webCam = CameraServer.startAutomaticCapture();
    webCam.setVideoMode(PixelFormat.kMJPEG, 400, 225, 30);
    tab.add(webCam).withPosition(0, 0).withSize(9, 5);
  }

  private VideoCamera getLimelight() {
    // HttpCamera httpCamera = new HttpCamera("Limelight Camera", \
    // "http://frcvision.local:1181/stream.mjpg"); //TODO fix
    return null;
  }
}
