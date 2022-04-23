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
const yaml = require("js-yaml");
const os = require("os");
const path = require("path");
const readline = require("readline");
const shell_quote = require("shell-quote");
const tmp = require("tmp");
const util = require("util");
const vscode = require("vscode");
const extension = require("../../../../extension");
const ros_1 = require("../../../../ros/ros");
const promisifiedExec = util.promisify(child_process.exec);
class LaunchResolver {
    // tslint:disable-next-line: max-line-length
    resolveDebugConfigurationWithSubstitutedVariables(folder, config, token) {
        return __awaiter(this, void 0, void 0, function* () {
            if (!path.isAbsolute(config.target) || (path.extname(config.target) !== ".launch" && path.extname(config.target) !== ".test")) {
                throw new Error("Launch request requires an absolute path as target.");
            }
            const delay = ms => new Promise(res => setTimeout(res, ms));
            // Manage the status of the ROS core, starting one if not present
            // The ROS core will continue to run until the VSCode window is closed
            if ((yield ros_1.rosApi.getCoreStatus()) == false) {
                console.log("ROS Core is not active, attempting to start automatically");
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
                console.log("Waited " + timeWaited + " for ROS Core to start");
                if (timeWaited >= timeout_ms) {
                    throw new Error('Timed out (' + timeWaited / 1000 + ' seconds) waiting for ROS Core to start. Start ROSCore manually to avoid this error.');
                }
            }
            const rosExecOptions = {
                env: yield extension.resolvedEnv(),
            };
            // If the configuration has arguments,
            let configArgs = "";
            if (config.arguments) {
                configArgs = config.arguments.join(' ');
            }
            let result = yield promisifiedExec(`roslaunch --dump-params ${config.target} ${configArgs}`, rosExecOptions);
            if (result.stderr) {
                throw (new Error(`Error from roslaunch:\r\n ${result.stderr}`));
            }
            else if (result.stdout.length == 0) {
                throw (new Error(`roslaunch unexpectedly produced no output, please test by running \"roslaunch --dump-params ${config.target} ${configArgs}\" in a ros terminal.`));
            }
            const parameters = Object.keys(yaml.load(result.stdout));
            if (parameters && parameters.length) {
                // only call into rosparam when necessary
                const tmpFile = tmp.fileSync();
                fs.writeFile(`${tmpFile.name}`, result.stdout, (error) => __awaiter(this, void 0, void 0, function* () {
                    if (error) {
                        throw error;
                    }
                    yield promisifiedExec(`rosparam load ${tmpFile.name}`, rosExecOptions);
                    tmpFile.removeCallback();
                }));
            }
            result = yield promisifiedExec(`roslaunch --nodes ${config.target} ${configArgs}`, rosExecOptions);
            if (result.stderr) {
                throw (new Error(`Error from roslaunch:\r\n ${result.stderr}`));
            }
            else if (result.stdout.length == 0) {
                throw (new Error(`roslaunch unexpectedly produced no output, please test by running \"roslaunch --dump-params ${config.target} ${configArgs}\" in a ros terminal.`));
            }
            const nodes = result.stdout.trim().split(os.EOL);
            yield Promise.all(nodes.map((node) => {
                return promisifiedExec(`roslaunch --args ${node} ${config.target} ${configArgs}`, rosExecOptions);
            })).then((commands) => {
                commands.forEach((command, index) => __awaiter(this, void 0, void 0, function* () {
                    const launchRequest = this.generateLaunchRequest(nodes[index], command.stdout, config);
                    if (launchRequest != null) {
                        this.executeLaunchRequest(launchRequest, false);
                    }
                    else {
                        const process = child_process.exec(command.stdout, rosExecOptions, (err, out) => {
                            if (err) {
                                throw (new Error(`Error from ${command.stdout}:\r\n ${err}`));
                            }
                        });
                    }
                }));
            });
            // @todo: error handling for Promise.all
            // Return null as we have spawned new debug requests
            return null;
        });
    }
    generateLaunchRequest(nodeName, command, config) {
        let parsedArgs;
        const isWindows = os.platform() === "win32";
        if (isWindows) {
            // https://github.com/ros/ros_comm/pull/1809
            // escape backslash in file path
            parsedArgs = shell_quote.parse(command.replace(/[\\]/g, "\\$&"));
            parsedArgs = shell_quote.parse(parsedArgs[2].toString().replace(/[\\]/g, "\\$&"));
        }
        else {
            parsedArgs = shell_quote.parse(command);
        }
        const envConfig = {};
        while (parsedArgs) {
            // https://github.com/ros/ros_comm/pull/1809
            if (isWindows && parsedArgs[0].toString() === "set") {
                parsedArgs.shift();
            }
            if (parsedArgs[0].toString().includes("=")) {
                const arg = parsedArgs.shift().toString();
                envConfig[arg.substring(0, arg.indexOf("="))] = arg.substring(arg.indexOf("=") + 1);
                // https://github.com/ros/ros_comm/pull/1809
                // "&&" is treated as Object
                if (isWindows && parsedArgs[0] instanceof Object) {
                    parsedArgs.shift();
                }
            }
            else {
                break;
            }
        }
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
        // If no specific nodes specifed, attach to all unless specifically ignored., 
        if (config.attachDebugger == null ||
            config.attachDebugger.indexOf(executableName) != -1) {
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
                if (request.executable.toLowerCase().endsWith("python") ||
                    request.executable.toLowerCase().endsWith("python.exe")) {
                    const pythonScript = request.arguments.shift();
                    const pythonLaunchConfig = {
                        name: request.nodeName,
                        type: "python",
                        request: "launch",
                        program: pythonScript,
                        args: request.arguments,
                        env: request.env,
                        stopOnEntry: stopOnEntry,
                        justMyCode: false,
                    };
                    debugConfig = pythonLaunchConfig;
                }
                else if (request.executable.endsWith(".exe")) {
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