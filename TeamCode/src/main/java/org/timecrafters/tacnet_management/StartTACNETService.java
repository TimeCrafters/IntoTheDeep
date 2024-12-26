package org.timecrafters.tacnet_management;

import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.TimeCraftersConfigurationTool.library.tacnet.TACNETServerService;

import java.util.Objects;

@TeleOp(name = "Start TACNET Service", group = "TACNET")
public class StartTACNETService extends OpMode {
    @Override
    public void init() {
        Context appContext = hardwareMap.appContext;
        Intent tacnetIntent;
        tacnetIntent = new Intent("org.timecrafters.TimeCraftersConfigurationTool.tacnet.ACTION_START_SERVER");
        tacnetIntent.setPackage("org.timecrafters.TimeCraftersConfigurationTool");

        // If TimeCrafters Configuration Tool app isn't installed then fallback to alternative means
        // of running service that results in the service ending whenever the Robot Controller app
        // is stopped or re-installed.
        try {
            appContext.getPackageManager().getPackageInfo(Objects.requireNonNull(tacnetIntent.getPackage()), 0);
        } catch (PackageManager.NameNotFoundException e) {
            tacnetIntent = new Intent(appContext, TACNETServerService.class);
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            appContext.startForegroundService(tacnetIntent);
        } else {
            appContext.startService(tacnetIntent);
        }
    }

    @Override
    public void loop() {
        stop();
    }
}
