package org.firstinspires.ftc.teamcode.file;

import android.os.Environment;

import java.io.File;

public class TrajectoryFiles {
    public static String PATH_TO_DATA_FOLDER = String.format(
            "%s/FIRST/data",
            Environment
                    .getExternalStorageDirectory()
            .getAbsolutePath()
    );

    public static File loadFile(String path) {
        return new File(PATH_TO_DATA_FOLDER + "/" + path);
    }
}
