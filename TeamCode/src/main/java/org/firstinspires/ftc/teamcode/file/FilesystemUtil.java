package org.firstinspires.ftc.teamcode.file;

import android.os.Environment;

import java.io.File;

public class FilesystemUtil {
    public static String PATH_TO_FIRST_FOLDER = String.format(
            "%s/FIRST/data",
            Environment
                    .getExternalStorageDirectory()
                    .getAbsolutePath()
    );

    public static File loadFile(String path) { return new File(PATH_TO_FIRST_FOLDER + "/" + path); }

    public static File loadDataFolder() { return new File(PATH_TO_FIRST_FOLDER); }
}
