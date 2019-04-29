package frc.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;

import frc.robot.Constants;

/**
 * Logger
 */
public class Logger {

    public static void writeCrashLogger(Exception exception) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(Constants.Logger.FILE_PATH, true))) {
            writer.print(new Date().toString());

            if (exception != null) {
                writer.print(", ");
                exception.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void write(String name, String data) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(Constants.Logger.NAMED_FILE_PATH + name + ".txt", true))) {
            writer.print(new Date().toString());

            if (data != null) {
                writer.print(", ");
                writer.print(data);
            }

            writer.println();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
}