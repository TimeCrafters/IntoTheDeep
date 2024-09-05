package org.timecrafters.TimeCraftersConfigurationTool.library.backend;

import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Action;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Configuration;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Group;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Presets;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Variable;

import java.util.ArrayList;
import java.util.Date;

public class Config {
    private String name;
    private Configuration configuration;
    private ArrayList<Group> groups;
    private Presets presets;

    public Config(String name) {
        this.name = name;
        this.configuration = new Configuration(new Date(), new Date(), TAC.CONFIG_SPEC_VERSION, 0);
        groups = new ArrayList<>();
        presets = new Presets(new ArrayList<Group>(), new ArrayList<Action>());
    }

    public Config(Configuration configuration, ArrayList<Group> groups, Presets presets) {
        this.configuration = configuration;
        this.groups = groups;
        this.presets = presets;
    }

    public String getName() { return name; }
    public void setName(String name) { this.name = name; }

    public Configuration getConfiguration() {
        return configuration;
    }

    public Presets getPresets() {
        return presets;
    }

    public ArrayList<Group> getGroups() {
        return groups;
    }

    /**
     * Syncs the values of Variables from reloaded config
     * @param newConfig
     */
    public void syncVariables(Config newConfig) {
        for (Group group : groups) {
            for (Action action : group.getActions()) {

                for (Group newGroup : newConfig.groups) {
                    for (Action newAction : newGroup.getActions()) {

                        if (group.name.trim().equals(newGroup.name.trim()) && action.name.trim().equals(newAction.name.trim())) {
                            for (Variable variable : action.getVariables()) {
                                for (Variable newVariable : newAction.getVariables()) {

                                    if (variable.name.trim().equals(newVariable.name.trim())) {
                                        variable.setValue(newVariable.rawValue());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
