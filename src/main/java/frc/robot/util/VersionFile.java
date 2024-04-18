package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Scanner;

public class VersionFile {

    private static VersionFile instance = null;
    private final String miniVersionString;
    private boolean isStringOnShuffleboard = false;

    private VersionFile() {
        File versionFileMini = new File(Filesystem.getDeployDirectory(), Constants.Dashboard.VERSION_FILE_NAME);
        StringBuilder builder = new StringBuilder();
        if (versionFileMini.canRead()) {
            try (Scanner reader = new Scanner(new FileReader(versionFileMini))) {
                while (reader.hasNext()) {
                    builder.append(reader.nextLine() + "\n");
                }
            } catch (FileNotFoundException e) {
                // TODO: add log statement, remove DS error
                DriverStation.reportError("Exception occured while trying to read version string from file", false);
                miniVersionString = "Version string unreadable: FileNotFound exception occurred.";
                return;
            }
            miniVersionString = builder.toString();
        } else {
            miniVersionString = "Version string unreadable: File unreadable";
        }
    }

    public static VersionFile getInstance() {
        if (instance == null) {
            instance = new VersionFile();
        }
        return instance;
    }

    /**
     * Place the version string in a shuffleboard string view in a tab with the given name. This
     * method only needs to be called once. The method is internally protected from multiple calls, so
     * calls subsequent to the first will do nothing. The UI elements will be placed such that they
     * occupy the area of the tab from (0, 0) to (3, 5).
     *
     * @param tabTitle The title of the shuffleboard tab for the version information to appear on.
     */
    public void putToDashboard(String tabTitle) {
        if (!isStringOnShuffleboard) {
            ShuffleboardTab t = Shuffleboard.getTab(tabTitle);
            t.addString("Robot Version: ", () -> miniVersionString).withSize(3, 1);
            isStringOnShuffleboard = true;
        }
    }

    /** Place the version string in a shuffleboard string view in a tab called "Version". */
    public void putToDashboard() {
        putToDashboard("Version");
    }

    @Override
    public String toString() {
        return miniVersionString;
    }
}
