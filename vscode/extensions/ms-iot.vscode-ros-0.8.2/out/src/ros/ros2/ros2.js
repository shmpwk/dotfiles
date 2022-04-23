"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
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
var __asyncValues = (this && this.__asyncValues) || function (o) {
    if (!Symbol.asyncIterator) throw new TypeError("Symbol.asyncIterator is not defined.");
    var m = o[Symbol.asyncIterator], i;
    return m ? m.call(o) : (o = typeof __values === "function" ? __values(o) : o[Symbol.iterator](), i = {}, verb("next"), verb("throw"), verb("return"), i[Symbol.asyncIterator] = function () { return this; }, i);
    function verb(n) { i[n] = o[n] && function (v) { return new Promise(function (resolve, reject) { v = o[n](v), settle(resolve, reject, v.done, v.value); }); }; }
    function settle(resolve, reject, d, v) { Promise.resolve(v).then(function(v) { resolve({ value: v, done: d }); }, reject); }
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.ROS2 = void 0;
const child_process = require("child_process");
const fs = require("fs");
const os = require("os");
const path = require("path");
const util = require("util");
const daemon = require("./daemon");
const ros2_monitor = require("./ros2-monitor");
const ros_utils = require("../utils");
const promisifiedExists = util.promisify(fs.exists);
const promisifiedExec = util.promisify(child_process.exec);
class ROS2 {
    setContext(context, env) {
        this.context = context;
        this.env = env;
    }
    getPackageNames() {
        return new Promise((resolve, reject) => child_process.exec("ros2 pkg list", { env: this.env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    return line;
                }));
                resolve(lines);
            }
            else {
                reject(err);
            }
        }));
    }
    getPackages() {
        var e_1, _a;
        return __awaiter(this, void 0, void 0, function* () {
            const packages = {};
            const { stdout } = child_process.exec("ros2 pkg list", { env: this.env });
            let chucks = "";
            try {
                for (var stdout_1 = __asyncValues(stdout), stdout_1_1; stdout_1_1 = yield stdout_1.next(), !stdout_1_1.done;) {
                    const chuck = stdout_1_1.value;
                    chucks += chuck;
                }
            }
            catch (e_1_1) { e_1 = { error: e_1_1 }; }
            finally {
                try {
                    if (stdout_1_1 && !stdout_1_1.done && (_a = stdout_1.return)) yield _a.call(stdout_1);
                }
                finally { if (e_1) throw e_1.error; }
            }
            chucks.split(os.EOL).map(((line) => {
                const packageName = line.trim();
                packages[packageName] = () => __awaiter(this, void 0, void 0, function* () {
                    var e_2, _a;
                    const { stdout } = yield child_process.exec(`ros2 pkg prefix --share ${packageName}`, { env: this.env });
                    let innerChucks = "";
                    try {
                        for (var stdout_2 = __asyncValues(stdout), stdout_2_1; stdout_2_1 = yield stdout_2.next(), !stdout_2_1.done;) {
                            const chuck = stdout_2_1.value;
                            innerChucks += chuck;
                        }
                    }
                    catch (e_2_1) { e_2 = { error: e_2_1 }; }
                    finally {
                        try {
                            if (stdout_2_1 && !stdout_2_1.done && (_a = stdout_2.return)) yield _a.call(stdout_2);
                        }
                        finally { if (e_2) throw e_2.error; }
                    }
                    return innerChucks.trim();
                });
            }));
            return packages;
        });
    }
    getIncludeDirs() {
        return __awaiter(this, void 0, void 0, function* () {
            const prefixPaths = [];
            if (this.env.AMENT_PREFIX_PATH) {
                prefixPaths.push(...this.env.AMENT_PREFIX_PATH.split(path.delimiter));
            }
            const includeDirs = [];
            for (const dir of prefixPaths) {
                const include = path.join(dir, "include");
                if (yield promisifiedExists(include)) {
                    includeDirs.push(include);
                }
            }
            return includeDirs;
        });
    }
    getWorkspaceIncludeDirs(workspaceDir) {
        return __awaiter(this, void 0, void 0, function* () {
            const includes = [];
            const opts = {
                env: this.env,
                cwd: workspaceDir
            };
            const result = yield promisifiedExec(`colcon list -p --base-paths "${workspaceDir}"`, opts);
            // error out if we see anything from stderr.
            if (result.stderr) {
                return includes;
            }
            // each line should be a path like `c:\ros2_ws\src\demos\demo_nodes_cpp`
            for (const line of result.stdout.split(os.EOL)) {
                const include = path.join(line, "include");
                if (yield promisifiedExists(include)) {
                    includes.push(include);
                }
            }
            return includes;
        });
    }
    findPackageExecutables(packageName) {
        return new Promise((resolve, reject) => child_process.exec(`ros2 pkg executables ${packageName}`, { env: this.env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    const info = line.split(" ");
                    if (info.length === 2) {
                        // each line should contain exactly 2 strings separated by 1 space
                        return info;
                    }
                }));
                const packageInfoReducer = (acc, cur) => {
                    const executableName = cur[1];
                    acc.push(executableName);
                    return acc;
                };
                resolve(lines.reduce(packageInfoReducer, []));
            }
            else {
                reject(err);
            }
        }));
    }
    findPackageLaunchFiles(packageName) {
        return __awaiter(this, void 0, void 0, function* () {
            const packages = yield this.getPackages();
            const packageBasePath = yield packages[packageName]();
            const command = (process.platform === "win32") ?
                `where /r "${packageBasePath}" *launch.py` :
                `find -L "${packageBasePath}" -type f -name *launch.py`;
            return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) => {
                err ? e(new Error('No launch files are found.')) : c(out.trim().split(os.EOL));
            }));
        });
    }
    findPackageTestFiles(packageName) {
        return __awaiter(this, void 0, void 0, function* () {
            // TODO: ROS2 rostest equivalent not implemented yet
            return new Promise((resolve, reject) => {
                resolve([]);
            });
        });
    }
    startCore() {
        return __awaiter(this, void 0, void 0, function* () {
            daemon.startDaemon();
        });
    }
    stopCore() {
        return __awaiter(this, void 0, void 0, function* () {
            daemon.stopDaemon();
        });
    }
    getCoreStatus() {
        return __awaiter(this, void 0, void 0, function* () {
            const ros2cliApi = new ros2_monitor.XmlRpcApi();
            return ros2cliApi.check();
        });
    }
    rosdep() {
        const terminal = ros_utils.createTerminal(this.context);
        terminal.sendText(`rosdep install --from-paths src --ignore-src -r -y`);
        return terminal;
    }
    activateCoreMonitor() {
        const coreStatusItem = new daemon.StatusBarItem();
        coreStatusItem.activate();
        return coreStatusItem;
    }
    showCoreMonitor() {
        return __awaiter(this, void 0, void 0, function* () {
            return ros2_monitor.launchMonitor(this.context);
        });
    }
    activateRosrun(packageName, executableName, argument) {
        const terminal = ros_utils.createTerminal(this.context);
        terminal.sendText(`ros2 run ${packageName} ${executableName} ${argument}`);
        return terminal;
    }
    activateRoslaunch(launchFilepath, argument) {
        const terminal = ros_utils.createTerminal(this.context);
        terminal.sendText(`ros2 launch ${launchFilepath} ${argument}`);
        return terminal;
    }
    activateRostest(launchFilepath, argument) {
        console.error("ROS2 rostest equivalent not implemented yet");
        return;
    }
}
exports.ROS2 = ROS2;
//# sourceMappingURL=ros2.js.map