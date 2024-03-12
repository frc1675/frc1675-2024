package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriverDashboard {

  public DriverDashboard(RobotContext robotContext) {
    ShuffleboardTab tab = Shuffleboard.getTab("Driver");
    VideoCamera webCam = getWebCam();
    tab.add(webCam).withPosition(0, 0).withSize(5, 5);

    // VideoCamera limelightCamera = getLimelight();
  }

  private VideoCamera getWebCam() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    return camera;

    // get and set camera flavor:
      // camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 60);
      // VideoMode currentVideoMode = camera.getVideoMode();
      // currentVideoMode.fps -> fps
      // currentVideoMode.height -> height ...etc
  }

  private VideoCamera getLimelight() {
    // HttpCamera httpCamera = new HttpCamera("Limelight Camera", \
    // "http://frcvision.local:1181/stream.mjpg"); //TODO fix
    return null;
  }
}
