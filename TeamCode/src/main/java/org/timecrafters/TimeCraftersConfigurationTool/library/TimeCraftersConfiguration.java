package org.timecrafters.TimeCraftersConfigurationTool.library;

import android.os.Environment;
import android.os.FileObserver;

import androidx.annotation.Nullable;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.timecrafters.TimeCraftersConfigurationTool.library.backend.Config;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.Settings;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.TAC;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Action;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Configuration;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Group;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Presets;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Variable;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.ActionDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.ActionSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.ConfigDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.ConfigSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.ConfigurationDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.ConfigurationSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.GroupDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.GroupSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.PresetsDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.PresetsSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.SettingsDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.SettingsSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.VariableDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.library.serializers.VariableSerializer;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;

public class TimeCraftersConfiguration {
    private static final String TAG = "TCT|TCConfig";
    private Config config;
    private boolean liveReload = false;
    private FileObserver fileObserver;

    public TimeCraftersConfiguration() {
        Settings settings = loadSettings();
        this.config = loadConfig(settings.config);
    }

    public TimeCraftersConfiguration(String configName) {
        this.config = loadConfig(configName);
    }

    public Config getConfig() {
        return config;
    }

    public void enableLiveReload(boolean b) {
        liveReload = b;
    }

    public Group group(String groupName) {
        for (final Group group : config.getGroups()) {
            if (group.name.trim().equals(groupName.trim())) {
                return group;
            }
        }

        throw(new TimeCraftersConfigurationError("Failed to find a group named:\"" + groupName.trim() + "\" in config \"" + config.getName() + "\""));
    }

    public Action action(String groupName, String actionName) {
        final Group group = group(groupName);

        for (Action action : group.getActions()) {
            if (action.name.trim().equals(actionName.trim())) {
                return action;
            }
        }

        throw(new TimeCraftersConfigurationError("Failed to find an action named:\"" + actionName.trim() + "\" in group \"" + groupName.trim() + "\" in config \"" + config.getName() + "\""));
    }

    public Variable variable(String groupName, String actionName, String variableName) {
        final Action action = action(groupName, actionName);

        for (Variable variable : action.getVariables()) {
            if (variable.name.trim().equals(variableName.trim())) {
                return variable;
            }
        }

        throw(new TimeCraftersConfigurationError("Failed to find a variable named \"" + variableName.trim() + "\" in action:\"" + actionName.trim() +
                "\" in group \"" + groupName.trim() + "\" in config \"" + config.getName() + "\""));
    }

    public String getRootPath() {
        //                             Ducktape :|
        return String.format("%s%s", Environment.getExternalStorageDirectory() + "/Android/data/org.timecrafters.TimeCraftersConfigurationTool/files", File.separator + "TimeCrafters_Configuration_Tool");
    }

    public String getConfigsPath() {
        return String.format("%s%s", getRootPath(),  File.separator + "/configs");
    }

    public String getSettingsPath() {
        return String.format("%s%s", getRootPath(),  File.separator + "settings.json");
    }

    private Settings loadSettings() {
        File settingsFile = new File(TAC.SETTINGS_PATH);

        if (!settingsFile.exists()) {
            settingsFile = new File(getSettingsPath());

            if (!settingsFile.exists()) {
                throw( new TimeCraftersConfigurationError("Unable to load settings.json, file does not exist!") );
            }
        }

        try {
            return gsonForSettings().fromJson(new FileReader(settingsFile), Settings.class);
        } catch (FileNotFoundException e) {
            throw( new TimeCraftersConfigurationError("Unable to load settings.json") );
        }
    }

    private Config loadConfig(String name) {
        if (name.equals("")) {
            throw(new TimeCraftersConfigurationError("Cannot load a config with an empty name!"));
        }

        String path = TAC.CONFIGS_PATH + File.separator + name + ".json";
        File configFile = new File(path);

        // Try looking in "new" location for config files
        if (!configFile.exists()) {
            configFile = new File(getConfigsPath() + File.separator + name + ".json");
        }

        if (configFile.exists() && configFile.isFile()) {
            try {
                Config config = gsonForConfig().fromJson(new FileReader(configFile), Config.class);
                config.setName(name);

                // Needs debugging...
                /* final TimeCraftersConfiguration self = this;
                if (fileObserver == null) {
                    fileObserver = new FileObserver(configFile) {
                        @Override
                        public void onEvent(int event, @Nullable String filePath) {
                            if (self.liveReload && event == CLOSE_WRITE) {
                                Config newConfig = loadConfig(path);

                                self.config.syncVariables(newConfig);
                            }
                        }
                    };
                    fileObserver.startWatching();
                } */

                return config;
            } catch (FileNotFoundException e) {
                e.printStackTrace();
                throw(new TimeCraftersConfigurationError("Unable to find a config file named \"" + name + "\""));
            }
        } else {
            throw(new TimeCraftersConfigurationError("Unable to find a config file named \"" + name + "\""));
        }
    }

    private Gson gsonForSettings() {
        return new GsonBuilder()
                .registerTypeAdapter(Settings.class, new SettingsSerializer())
                .registerTypeAdapter(Settings.class, new SettingsDeserializer())
                .create();
    }

    public Gson gsonForConfig() {
        return new GsonBuilder()
                .registerTypeAdapter(Config.class, new ConfigSerializer())
                .registerTypeAdapter(Config.class, new ConfigDeserializer())

                .registerTypeAdapter(Configuration.class, new ConfigurationSerializer())
                .registerTypeAdapter(Configuration.class, new ConfigurationDeserializer())

                .registerTypeAdapter(Group.class, new GroupSerializer())
                .registerTypeAdapter(Group.class, new GroupDeserializer())

                .registerTypeAdapter(Action.class, new ActionSerializer())
                .registerTypeAdapter(Action.class, new ActionDeserializer())

                .registerTypeAdapter(Variable.class, new VariableSerializer())
                .registerTypeAdapter(Variable.class, new VariableDeserializer())

                .registerTypeAdapter(Presets.class, new PresetsSerializer())
                .registerTypeAdapter(Presets.class, new PresetsDeserializer())
                .create();
    }
}
