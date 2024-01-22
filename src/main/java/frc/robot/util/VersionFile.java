package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Scanner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class VersionFile {

    private static VersionFile instance = new VersionFile();
    private final String versionString;
    private boolean stringOnShuffleboard = false;

    public static VersionFile getInstance() {
        return instance;
    }

    private VersionFile() {
        File versionFile = new File(Filesystem.getDeployDirectory(), Constants.Dashboard.VERSION_FILE_NAME);
        StringBuilder builder = new StringBuilder();
        if (versionFile.canRead()) {
            try (Scanner reader = new Scanner(new FileReader(versionFile))) {

                while (reader.hasNext()) {
                    builder.append(reader.nextLine() + "\n");
                }

            } catch (FileNotFoundException e) {
                // TODO: add log statement, remove DS error
                DriverStation.reportError("Exception occured while trying to read version string from file", false);
                versionString = "Version string unreadable: FileNotFound exception occurred.";
                return;
            }
            versionString = builder.toString();

        } else {
            versionString = "Version string unreadable: File unreadable.";
        }

    }

    /**
     * Place the version string in a shuffleboard string view in a tab with the
     * given name.
     * This method only needs to be called once. The method is internally protected
     * from multiple calls, so calls subsequent to the first will do nothing.
     * The UI elements will be placed such that they occupy the area of the tab from
     * (0, 0) to (3, 5).
     * 
     * @param tabTitle The title of the shuffleboard tab for the version information
     *                 to appear on.
     */
    public void putToDashboard(String tabTitle) {
        if (!stringOnShuffleboard) {
            ShuffleboardTab t = Shuffleboard.getTab(tabTitle);
            String[] s = versionString.split("\n");
            t.addString("Commit", () -> s[0]).withPosition(0, 0).withSize(5, 1);
            t.addString("Branch", () -> s[1]).withPosition(0, 1).withSize(5, 1);
            t.addString("Uncommitted Changes", () -> s[2]).withPosition(0, 2).withSize(5, 1);
            stringOnShuffleboard = true;
        }
    }

    /**
     * Place the version string in a shuffleboard string view in a tab called
     * "Version".
     */
    public void putToDashboard() {
        putToDashboard("Version");
    }

    @Override
    public String toString() {
        return versionString;
    }

}
