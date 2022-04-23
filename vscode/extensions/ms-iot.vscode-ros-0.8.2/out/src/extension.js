"use strict";
// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.deactivate = exports.activate = exports.Commands = exports.resolvedEnv = exports.onDidChangeEnv = exports.outputChannel = exports.extPath = exports.env = exports.setBaseDir = exports.baseDir = void 0;
const path = require("path");
const vscode = require("vscode");
const cpp_formatter = require("./cpp-formatter");
const pfs = require("./promise-fs");
const telemetry = require("./telemetry-helper");
const vscode_utils = require("./vscode-utils");
const buildtool = require("./build-tool/build-tool");
const ros_build_utils = require("./ros/build-env-utils");
const ros_cli = require("./ros/cli");
const ros_utils = require("./ros/utils");
const ros_1 = require("./ros/ros");
const previewManager_1 = require("./urdfPreview/previewManager");
const debug_manager = require("./debugger/manager");
const debug_utils = require("./debugger/utils");
const ros_shell_1 = require("./build-tool/ros-shell");
function setBaseDir(dir) {
    exports.baseDir = dir;
}
exports.setBaseDir = setBaseDir;
let onEnvChanged = new vscode.EventEmitter();
/**
 * Triggered when the env is soured.
 */
exports.onDidChangeEnv = onEnvChanged.event;
function resolvedEnv() {
    return __awaiter(this, void 0, void 0, function* () {
        if (exports.env === undefined) { // Env reload in progress
            yield debug_utils.oneTimePromiseFromEvent(exports.onDidChangeEnv, () => exports.env !== undefined);
        }
        return exports.env;
    });
}
exports.resolvedEnv = resolvedEnv;
/**
 * Subscriptions to dispose when the environment is changed.
 */
let subscriptions = [];
var Commands;
(function (Commands) {
    Commands["CreateCatkinPackage"] = "ros.createCatkinPackage";
    Commands["CreateTerminal"] = "ros.createTerminal";
    Commands["GetDebugSettings"] = "ros.getDebugSettings";
    Commands["Rosrun"] = "ros.rosrun";
    Commands["Roslaunch"] = "ros.roslaunch";
    Commands["Rostest"] = "ros.rostest";
    Commands["Rosdep"] = "ros.rosdep";
    Commands["ShowCoreStatus"] = "ros.showCoreStatus";
    Commands["StartRosCore"] = "ros.startCore";
    Commands["TerminateRosCore"] = "ros.stopCore";
    Commands["UpdateCppProperties"] = "ros.updateCppProperties";
    Commands["UpdatePythonPath"] = "ros.updatePythonPath";
    Commands["PreviewURDF"] = "ros.previewUrdf";
})(Commands = exports.Commands || (exports.Commands = {}));
function activate(context) {
    return __awaiter(this, void 0, void 0, function* () {
        let init = vscode.window.withProgress({
            location: vscode.ProgressLocation.Notification,
            title: "ROS Extension Initializing...",
            cancellable: false
        }, (progress, token) => __awaiter(this, void 0, void 0, function* () {
            const reporter = telemetry.getReporter();
            exports.extPath = context.extensionPath;
            exports.outputChannel = vscode_utils.createOutputChannel();
            context.subscriptions.push(exports.outputChannel);
            // Determine if we're in a catkin workspace.
            let buildToolDetected = yield buildtool.determineBuildTool(vscode.workspace.rootPath);
            // Activate components when the ROS env is changed.
            context.subscriptions.push((0, exports.onDidChangeEnv)(activateEnvironment.bind(null, context, buildToolDetected)));
            // Activate components which don't require the ROS env.
            context.subscriptions.push(vscode.languages.registerDocumentFormattingEditProvider("cpp", new cpp_formatter.CppFormatter()));
            previewManager_1.default.INSTANCE.setContext(context);
            // Source the environment, and re-source on config change.
            let config = vscode_utils.getExtensionConfiguration();
            context.subscriptions.push(vscode.workspace.onDidChangeConfiguration(() => {
                const updatedConfig = vscode_utils.getExtensionConfiguration();
                const fields = Object.keys(config).filter(k => !(config[k] instanceof Function));
                const changed = fields.some(key => updatedConfig[key] !== config[key]);
                if (changed) {
                    sourceRosAndWorkspace();
                }
                config = updatedConfig;
            }));
            yield sourceRosAndWorkspace().then(() => {
                vscode.window.registerWebviewPanelSerializer('urdfPreview', previewManager_1.default.INSTANCE);
            });
            reporter.sendTelemetryActivate();
            return {
                getBaseDir: () => exports.baseDir,
                getEnv: () => exports.env,
                onDidChangeEnv: (listener, thisArg) => (0, exports.onDidChangeEnv)(listener, thisArg),
            };
        }));
        return yield init;
    });
}
exports.activate = activate;
function deactivate() {
    return __awaiter(this, void 0, void 0, function* () {
        subscriptions.forEach(disposable => disposable.dispose());
        yield telemetry.clearReporter();
    });
}
exports.deactivate = deactivate;
function ensureErrorMessageOnException(callback) {
    return __awaiter(this, void 0, void 0, function* () {
        try {
            yield callback();
        }
        catch (err) {
            vscode.window.showErrorMessage(err.message);
        }
    });
}
/**
 * Activates components which require a ROS env.
 */
function activateEnvironment(context, buildToolDetected) {
    // Clear existing disposables.
    while (subscriptions.length > 0) {
        subscriptions.pop().dispose();
    }
    if (typeof exports.env.ROS_DISTRO === "undefined") {
        return;
    }
    if (typeof exports.env.ROS_VERSION === "undefined") {
        return;
    }
    // http://www.ros.org/reps/rep-0149.html#environment-variables
    // Learn more about ROS_VERSION definition.
    (0, ros_1.selectROSApi)(exports.env.ROS_VERSION);
    ros_1.rosApi.setContext(context, exports.env);
    subscriptions.push(ros_1.rosApi.activateCoreMonitor());
    if (buildToolDetected) {
        subscriptions.push(...buildtool.BuildTool.registerTaskProvider());
    }
    subscriptions.push(...(0, ros_shell_1.registerRosShellTaskProvider)());
    debug_manager.registerRosDebugManager(context);
    // register plugin commands
    subscriptions.push(vscode.commands.registerCommand(Commands.CreateTerminal, () => {
        ensureErrorMessageOnException(() => {
            ros_utils.createTerminal(context);
        });
    }), vscode.commands.registerCommand(Commands.GetDebugSettings, () => {
        ensureErrorMessageOnException(() => {
            return debug_utils.getDebugSettings(context);
        });
    }), vscode.commands.registerCommand(Commands.ShowCoreStatus, () => {
        ensureErrorMessageOnException(() => {
            ros_1.rosApi.showCoreMonitor();
        });
    }), vscode.commands.registerCommand(Commands.StartRosCore, () => {
        ensureErrorMessageOnException(() => {
            ros_1.rosApi.startCore();
        });
    }), vscode.commands.registerCommand(Commands.TerminateRosCore, () => {
        ensureErrorMessageOnException(() => {
            ros_1.rosApi.stopCore();
        });
    }), vscode.commands.registerCommand(Commands.UpdateCppProperties, () => {
        ensureErrorMessageOnException(() => {
            return ros_build_utils.updateCppProperties(context);
        });
    }), vscode.commands.registerCommand(Commands.UpdatePythonPath, () => {
        ensureErrorMessageOnException(() => {
            ros_build_utils.updatePythonPath(context);
        });
    }), vscode.commands.registerCommand(Commands.Rosrun, () => {
        ensureErrorMessageOnException(() => {
            return ros_cli.rosrun(context);
        });
    }), vscode.commands.registerCommand(Commands.Roslaunch, () => {
        ensureErrorMessageOnException(() => {
            return ros_cli.roslaunch(context);
        });
    }), vscode.commands.registerCommand(Commands.Rostest, () => {
        ensureErrorMessageOnException(() => {
            return ros_cli.rostest(context);
        });
    }), vscode.commands.registerCommand(Commands.PreviewURDF, () => {
        ensureErrorMessageOnException(() => {
            previewManager_1.default.INSTANCE.preview(vscode.window.activeTextEditor.document.uri);
        });
    }));
    // Register commands dependent on a workspace
    if (buildToolDetected) {
        subscriptions.push(vscode.commands.registerCommand(Commands.CreateCatkinPackage, () => {
            ensureErrorMessageOnException(() => {
                return buildtool.BuildTool.createPackage(context);
            });
        }), vscode.commands.registerCommand(Commands.Rosdep, () => {
            ensureErrorMessageOnException(() => {
                ros_1.rosApi.rosdep();
            });
        }), vscode.tasks.onDidEndTask((event) => {
            if (buildtool.isROSBuildTask(event.execution.task)) {
                sourceRosAndWorkspace();
            }
        }));
    }
    else {
        subscriptions.push(vscode.commands.registerCommand(Commands.CreateCatkinPackage, () => {
            vscode.window.showErrorMessage(`${Commands.CreateCatkinPackage} requires a ROS workspace to be opened`);
        }), vscode.commands.registerCommand(Commands.Rosdep, () => {
            vscode.window.showErrorMessage(`${Commands.Rosdep} requires a ROS workspace to be opened`);
        }));
    }
    // Generate config files if they don't already exist, but only for catkin workspaces
    if (buildToolDetected) {
        ros_build_utils.createConfigFiles();
    }
}
/**
 * Loads the ROS environment, and prompts the user to select a distro if required.
 */
function sourceRosAndWorkspace() {
    return __awaiter(this, void 0, void 0, function* () {
        exports.env = undefined;
        const kWorkspaceConfigTimeout = 30000; // ms
        let setupScriptExt;
        if (process.platform === "win32") {
            setupScriptExt = ".bat";
        }
        else {
            setupScriptExt = ".bash";
        }
        const config = vscode_utils.getExtensionConfiguration();
        let isolateEnvironment = config.get("isolateEnvironment", "");
        if (!isolateEnvironment) {
            // Capture the host environment unless specifically isolated
            exports.env = process.env;
        }
        let rosSetupScript = config.get("rosSetupScript", "");
        // If the workspace setup script is not set, try to find the ROS setup script in the environment
        let attemptWorkspaceDiscovery = true;
        if (rosSetupScript) {
            // Try to support cases where the setup script doesn't make sense on different environments, such as host vs container.
            if (yield pfs.exists(rosSetupScript)) {
                try {
                    exports.env = yield ros_utils.sourceSetupFile(rosSetupScript, exports.env);
                    attemptWorkspaceDiscovery = false;
                }
                catch (err) {
                    vscode.window.showErrorMessage(`A workspace setup script was provided in the configuration, but could not source "${rosSetupScript}". Attempting standard discovery.`);
                }
            }
        }
        if (attemptWorkspaceDiscovery) {
            let distro = config.get("distro", "");
            // Is there a distro defined either by setting or environment?
            if (!distro && !process.env.ROS_DISTRO) {
                // No? Try to find one.
                const installedDistros = yield ros_utils.getDistros();
                if (!installedDistros.length) {
                    throw new Error("ROS has not been found on this system.");
                }
                else if (installedDistros.length === 1) {
                    // if there is only one distro installed, directly choose it
                    config.update("distro", installedDistros[0]);
                }
                else {
                    const message = "Unable to determine ROS distribution, please configure this workspace by adding \"ros.distro\": \"<ROS Distro>\" in settings.json";
                    yield vscode.window.setStatusBarMessage(message, kWorkspaceConfigTimeout);
                }
            }
            if (distro) {
                let setupScript;
                try {
                    let globalInstallPath;
                    if (process.platform === "win32") {
                        globalInstallPath = path.join("C:", "opt", "ros", `${distro}`, "x64");
                    }
                    else {
                        globalInstallPath = path.join("/", "opt", "ros", `${distro}`);
                    }
                    setupScript = path.format({
                        dir: globalInstallPath,
                        name: "setup",
                        ext: setupScriptExt,
                    });
                    exports.env = yield ros_utils.sourceSetupFile(setupScript, exports.env);
                }
                catch (err) {
                    vscode.window.showErrorMessage(`Could not source ROS setup script at "${setupScript}".`);
                }
            }
            else if (process.env.ROS_DISTRO) {
                exports.env = process.env;
            }
        }
        // Source the workspace setup over the top.
        // TODO: we should test what's the build tool (catkin vs colcon).
        let workspaceOverlayPath;
        workspaceOverlayPath = path.join(`${exports.baseDir}`, "devel_isolated");
        if (!(yield pfs.exists(workspaceOverlayPath))) {
            workspaceOverlayPath = path.join(`${exports.baseDir}`, "devel");
        }
        if (!(yield pfs.exists(workspaceOverlayPath))) {
            workspaceOverlayPath = path.join(`${exports.baseDir}`, "install");
        }
        let wsSetupScript = path.format({
            dir: workspaceOverlayPath,
            name: "setup",
            ext: setupScriptExt,
        });
        if (exports.env && typeof exports.env.ROS_DISTRO !== "undefined" && (yield pfs.exists(wsSetupScript))) {
            try {
                exports.env = yield ros_utils.sourceSetupFile(wsSetupScript, exports.env);
            }
            catch (_err) {
                vscode.window.showErrorMessage("Failed to source the workspace setup file.");
            }
        }
        // Notify listeners the environment has changed.
        onEnvChanged.fire();
    });
}
//# sourceMappingURL=extension.js.map