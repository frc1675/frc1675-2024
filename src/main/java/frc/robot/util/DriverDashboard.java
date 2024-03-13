package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriverDashboard {
  private VideoCamera webCam;

  public DriverDashboard(RobotContext robotContext) {
    ShuffleboardTab tab = Shuffleboard.getTab("Driver");
    webCam = CameraServer.startAutomaticCapture();
    webCamInit(webCam);
    tab.add(webCam).withPosition(0, 0).withSize(5, 5);

    // VideoCamera limelightCamera = getLimelight();
  }

  private void webCamInit(VideoCamera camera) {
    // get and set camera flavor:
      // camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 60);
      VideoMode currentVideoMode = camera.getVideoMode();
      System.out.print("FPS" + currentVideoMode.fps + ";" + "WIDTH x HEIGHT" + currentVideoMode.width + " x " + currentVideoMode.height);
      System.out.print("FPS" + currentVideoMode.fps + ";" + "WIDTH x HEIGHT" + currentVideoMode.width + " x " + currentVideoMode.height);
      System.out.print("FPS" + currentVideoMode.fps + ";" + "WIDTH x HEIGHT" + currentVideoMode.width + " x " + currentVideoMode.height);
      System.out.print("FPS" + currentVideoMode.fps + ";" + "WIDTH x HEIGHT" + currentVideoMode.width + " x " + currentVideoMode.height);
      System.out.print("FPS" + currentVideoMode.fps + ";" + "WIDTH x HEIGHT" + currentVideoMode.width + " x " + currentVideoMode.height);
  }

  private VideoCamera getLimelight() {
    // HttpCamera httpCamera = new HttpCamera("Limelight Camera", \
    // "http://frcvision.local:1181/stream.mjpg"); //TODO fix
    return null;
  }
}
