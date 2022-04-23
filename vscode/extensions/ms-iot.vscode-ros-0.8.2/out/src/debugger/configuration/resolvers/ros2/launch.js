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
exports.LaunchResolver = void 0;
const child_process = require("child_process");
const fs = require("fs");
const os = require("os");
const path = require("path");
const readline = require("readline");
const shell_quote = require("shell-quote");
const util = require("util");
const vscode = require("vscode");
const extension = require("../../../../extension");
const ros_1 = require("../../../../ros/ros");
const promisifiedExec = util.promisify(child_process.exec);
function getExtensionFilePath(extensionFile) {
    return path.resolve(extension.extPath, extensionFile);
}
class LaunchResolver {
    // tslint:disable-next-line: max-line-length
    resolveDebugConfigurationWithSubstitutedVariables(folder, config, token) {
        return __awaiter(this, void 0, void 0, function* () {
            if (!path.isAbsolute(config.target)) {
                throw new Error("Launch request requires an absolute path as target.");
            }
            else if (path.extname(config.target) !== ".py" && path.extname(config.target) !== ".xml") {
                throw new Error("Launch request requires an extension '.py' or '.xml' as target.");
            }
            const delay = ms => new Promise(res => setTimeout(res, ms));
            // Manage the status of the ROS2 Daemon, starting one if not present
            if ((yield ros_1.rosApi.getCoreStatus()) == false) {
                console.log("ROS Daemon is not active, attempting to start automatically");
                ros_1.rosApi.startCore();
                // Wait for the core to start up to a timeout
                const timeout_ms = 30000;
                const interval_ms = 100;
                let timeWaited = 0;
                while ((yield ros_1.rosApi.getCoreStatus()) == false &&
                    timeWaited < timeout_ms) {
                    timeWaited += interval_ms;
                    yield delay(interval_ms);
                }
                console.log("Waited " + timeWaited + " for ROS2 Daemon to start");
                if (timeWaited >= timeout_ms) {
                    throw new Error('Timed out (' + timeWaited / 1000 + ' seconds) waiting for ROS2 Daemon to start. Start ROS2 Daemon manually to avoid this error.');
                }
            }
            const rosExecOptions = {
                env: Object.assign(Object.assign({}, yield extension.resolvedEnv()), config.env),
            };
            console.log("Executing dumper with the following environment:");
            console.log(rosExecOptions.env);
            let ros2_launch_dumper = getExtensionFilePath(path.join("assets", "scripts", "ros2_launch_dumper.py"));
            let args = [];
            if (config.arguments) {
                for (let arg of config.arguments) {
                    args.push(`"${arg}"`);
                }
            }
            let flatten_args = args.join(' ');
            let ros2_launch_dumper_cmdLine = (process.platform === "win32") ?
                `python ${ros2_launch_dumper} "${config.target}" ${flatten_args}` :
                `/usr/bin/env python3 ${ros2_launch_dumper} "${config.target}" ${flatten_args}`;
            let result = yield promisifiedExec(ros2_launch_dumper_cmdLine, rosExecOptions);
            if (result.stderr) {
                // Having stderr output is not nessesarily a problem, but it is useful for debugging
                console.log(`ROS2 launch processor produced stderr output:\r\n ${result.stderr}`);
            }
            if (result.stdout.length == 0) {
                throw (new Error(`ROS2 launch processor was unable to produce a node list.\r\n ${result.stderr}`));
            }
            let commands = result.stdout.split(os.EOL);
            commands.forEach((command) => __awaiter(this, void 0, void 0, function* () {
                // HACKHACK: To prevent processing spurious output as a command, only process lines that start with '\t' which is generated by the launch dumper.
                // This occurs commonly when running in a docker container with a host network connection. 
                // Something in ROS will output on stdout a notification of "arbitraily" selecting an interface. 
                if (!command || command[0] != '\t') {
                    return;
                }
                // trim to remove the tab character
                let process = command.trim().split(' ')[0];
                const launchRequest = this.generateLaunchRequest(process, command, config);
                if (launchRequest != null) {
                    this.executeLaunchRequest(launchRequest, false);
                }
                else {
                    const process = child_process.exec(command, rosExecOptions, (err, out) => {
                        if (err) {
                            throw (new Error(`Error from ${command}:\r\n ${err}`));
                        }
                    });
                }
            }));
            // @todo: error handling for Promise.all
            // Return null as we have spawned new debug requests
            return null;
        });
    }
    generateLaunchRequest(nodeName, command, config) {
        let parsedArgs;
        parsedArgs = shell_quote.parse(command);
        let executable = parsedArgs.shift().toString();
        // return rviz instead of rviz.exe, or spawner instead of spawner.py
        // This allows the user to run filter out genericly. 
        let executableName = path.basename(executable, path.extname(executable));
        // If this executable is just launched, don't attach a debugger.
        if (config.launch &&
            config.launch.indexOf(executableName) != -1) {
            return null;
        }
        // Filter shell scripts - just launch them
        //  https://github.com/ms-iot/vscode-ros/issues/474 
        let executableExt = path.extname(executable);
        if (executableExt &&
            ["bash", "sh", "bat", "cmd", "ps1"].includes(executableExt)) {
            return null;
        }
        // If a specific list of nodes is specified, then determine if this is one of them.
        // If no specific nodes specifed, attach to all unless specifically ignored.
        if (config.attachDebugger == null ||
            config.attachDebugger.indexOf(executableName) != -1) {
            const envConfig = config.env;
            const request = {
                nodeName: nodeName,
                executable: executable,
                arguments: parsedArgs.map((arg) => {
                    return arg.toString();
                }),
                cwd: ".",
                env: Object.assign(Object.assign({}, extension.env), envConfig),
                symbolSearchPath: config.symbolSearchPath,
                additionalSOLibSearchPath: config.additionalSOLibSearchPath,
                sourceFileMap: config.sourceFileMap
            };
            return request;
        }
        return null;
    }
    executeLaunchRequest(request, stopOnEntry) {
        return __awaiter(this, void 0, void 0, function* () {
            let debugConfig;
            if (os.platform() === "win32") {
                if (request.executable.toLowerCase().endsWith(".py")) {
                    const pythonLaunchConfig = {
                        name: request.nodeName,
                        type: "python",
                        request: "launch",
                        program: request.executable,
                        args: request.arguments,
                        env: request.env,
                        stopOnEntry: stopOnEntry,
                        justMyCode: false,
                    };
                    debugConfig = pythonLaunchConfig;
                }
                else if (request.executable.toLowerCase().endsWith(".exe")) {
                    const envConfigs = [];
                    for (const key in request.env) {
                        if (request.env.hasOwnProperty(key)) {
                            envConfigs.push({
                                name: key,
                                value: request.env[key],
                            });
                        }
                    }
                    const cppvsdbgLaunchConfig = {
                        name: request.nodeName,
                        type: "cppvsdbg",
                        request: "launch",
                        cwd: ".",
                        program: request.executable,
                        args: request.arguments,
                        environment: envConfigs,
                        stopAtEntry: stopOnEntry,
                        symbolSearchPath: request.symbolSearchPath,
                        sourceFileMap: request.sourceFileMap
                    };
                    debugConfig = cppvsdbgLaunchConfig;
                }
                if (!debugConfig) {
                    throw (new Error(`Failed to create a debug configuration!`));
                }
                const launched = yield vscode.debug.startDebugging(undefined, debugConfig);
                if (!launched) {
                    throw (new Error(`Failed to start debug session!`));
                }
            }
            else {
                try {
                    // this should be guaranteed by roslaunch
                    fs.accessSync(request.executable, fs.constants.X_OK);
                }
                catch (errNotExecutable) {
                    throw (new Error(`Error! ${request.executable} is not executable!`));
                }
                try {
                    // need to be readable to check shebang line
                    fs.accessSync(request.executable, fs.constants.R_OK);
                }
                catch (errNotReadable) {
                    throw (new Error(`Error! ${request.executable} is not readable!`));
                }
                const fileStream = fs.createReadStream(request.executable);
                const rl = readline.createInterface({
                    input: fileStream,
                    crlfDelay: Infinity,
                });
                // we only want to read 1 line to check for shebang line
                let linesToRead = 1;
                rl.on("line", (line) => __awaiter(this, void 0, void 0, function* () {
                    if (linesToRead <= 0) {
                        return;
                    }
                    linesToRead--;
                    if (!linesToRead) {
                        rl.close();
                    }
                    // look for Python in shebang line
                    if (line.startsWith("#!") && line.toLowerCase().indexOf("python") !== -1) {
                        const pythonLaunchConfig = {
                            name: request.nodeName,
                            type: "python",
                            request: "launch",
                            program: request.executable,
                            args: request.arguments,
                            env: request.env,
                            stopOnEntry: stopOnEntry,
                            justMyCode: false,
                        };
                        debugConfig = pythonLaunchConfig;
                    }
                    else {
                        const envConfigs = [];
                        for (const key in request.env) {
                            if (request.env.hasOwnProperty(key)) {
                                envConfigs.push({
                                    name: key,
                                    value: request.env[key],
                                });
                            }
                        }
                        const cppdbgLaunchConfig = {
                            name: request.nodeName,
                            type: "cppdbg",
                            request: "launch",
                            cwd: ".",
                            program: request.executable,
                            args: request.arguments,
                            environment: envConfigs,
                            stopAtEntry: stopOnEntry,
                            additionalSOLibSearchPath: request.additionalSOLibSearchPath,
                            sourceFileMap: request.sourceFileMap,
                            setupCommands: [
                                {
                                    text: "-enable-pretty-printing",
                                    description: "Enable pretty-printing for gdb",
                                    ignoreFailures: true
                                }
                            ]
                        };
                        debugConfig = cppdbgLaunchConfig;
                    }
                    if (!debugConfig) {
                        throw (new Error(`Failed to create a debug configuration!`));
                    }
                    const launched = yield vscode.debug.startDebugging(undefined, debugConfig);
                    if (!launched) {
                        throw (new Error(`Failed to start debug session!`));
                    }
                }));
            }
        });
    }
}
exports.LaunchResolver = LaunchResolver;
//# sourceMappingURL=launch.js.map