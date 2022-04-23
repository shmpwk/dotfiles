'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
const vscode = require("vscode");
const os_1 = require("os");
const lodash_1 = require("lodash");
const fs_1 = require("fs");
class ConfigManager {
    constructor() {
        this.config = {};
        if (ConfigManager._instance) {
            throw new Error("Error: Instantiation failed: Use ConfigManager.getInstance() instead of new.");
        }
        ConfigManager._instance = this;
    }
    static getInstance() {
        return ConfigManager._instance;
    }
    getConfig() {
        return this.config;
    }
    findCpplintPath(settings) {
        let cpplintPath = settings.get('cpplintPath', null);
        if (lodash_1.isNull(cpplintPath)) {
            let p = os_1.platform();
            if (p === 'win32') {
                // TODO: add win32 and win64 cpplint path
            }
            else if (p === 'linux' || p === 'darwin') {
                let attempts = ['/usr/local/bin/cpplint'];
                for (let index = 0; index < attempts.length; index++) {
                    if (fs_1.existsSync(attempts[index])) {
                        cpplintPath = attempts[index];
                        break;
                    }
                }
            }
        }
        return cpplintPath;
    }
    isSingleMode() {
        if (this.config['lintMode'] == 'single') {
            return true;
        }
        else {
            return false;
        }
    }
    isSupportLanguage(language) {
        if (this.config["languages"].indexOf(language) >= 0) {
            return true;
        }
        else {
            return false;
        }
    }
    initialize() {
        this.config = {};
        let settings = vscode.workspace.getConfiguration('cpplint');
        if (settings) {
            var cpplintPath = this.findCpplintPath(settings);
            if (!fs_1.existsSync(cpplintPath)) {
                vscode.window.showErrorMessage('Cpplint: Could not find cpplint executable');
            }
            this.config['cpplintPath'] = cpplintPath;
            var linelength = settings.get("lineLength", 80);
            this.config['lineLength'] = linelength;
            var lintmode = settings.get('lintMode', 'single');
            this.config['lintMode'] = lintmode;
            var excludes = settings.get('excludes', []);
            this.config['excludes'] = excludes;
            var filters = settings.get("filters", []);
            this.config["filters"] = filters;
            var root = settings.get("root", "");
            this.config["root"] = root;
            var languages = settings.get("languages", []);
            this.config["languages"] = languages;
            var extensions = settings.get("extensions", "");
            this.config["extensions"] = extensions;
            var headers = settings.get("headers", "");
            this.config["headers"] = headers;
            var repository = settings.get("repository", "");
            this.config["repository"] = repository;
            this.config["filters"].forEach(element => {
                if (element[0] != '-' && element[0] != '+') {
                    vscode.window.showErrorMessage("filter [" + element + '] must start with + or -, please check your settings');
                    return false;
                }
            });
            var verbose = settings.get("verbose", 0);
            this.config['verbose'] = verbose;
        }
        return this.config;
    }
}
ConfigManager._instance = new ConfigManager();
exports.ConfigManager = ConfigManager;
//# sourceMappingURL=configuration.js.map