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
exports.ROS1 = void 0;
const child_process = require("child_process");
const fs = require("fs");
const os = require("os");
const path = require("path");
const util = require("util");
const ros_core = require("./core-helper");
const ros_utils = require("../utils");
const promisifiedExists = util.promisify(fs.exists);
class ROS1 {
    constructor() {
        this.xmlRpcApi = null;
    }
    setContext(context, env) {
        this.context = context;
        this.env = env;
    }
    getPackageNames() {
        return this.getPackages().then((packages) => {
            return Object.keys(packages);
        });
    }
    getPackages() {
        return new Promise((resolve, reject) => child_process.exec("rospack list", { env: this.env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    const info = line.split(" ");
                    if (info.length === 2) {
                        // each line should contain exactly 2 strings separated by 1 space
                        return info;
                    }
                }));
                const packageInfoReducer = (acc, cur) => {
                    const k = cur[0];
                    const v = cur[1];
                    acc[k] = () => __awaiter(this, void 0, void 0, function* () {
                        return v;
                    });
                    return acc;
                };
                resolve(lines.reduce(packageInfoReducer, {}));
            }
            else {
                reject(err);
            }
        }));
    }
    getIncludeDirs() {
        const cmakePrefixPaths = [];
        if (this.env.hasOwnProperty("CMAKE_PREFIX_PATH")) {
            cmakePrefixPaths.push(...this.env.CMAKE_PREFIX_PATH.split(path.delimiter));
        }
        const includeDirs = [];
        const fsPromises = cmakePrefixPaths.map((dir) => {
            const include = path.join(dir, "include");
            return fs.promises.access(include, fs.constants.F_OK)
                .then(() => {
                includeDirs.push(include);
            })
                .catch(() => {
                // suppress exception if include folder does not exist
            });
        });
        return Promise.all(fsPromises).then(() => {
            return includeDirs;
        });
    }
    getWorkspaceIncludeDirs(workspaceDir) {
        return __awaiter(this, void 0, void 0, function* () {
            // Get all packages within the workspace that have an include directory
            const packages = yield this.getPackages();
            const filteredPackages = yield Object.values(packages).filter((packagePath) => __awaiter(this, void 0, void 0, function* () {
                const packageBasePath = yield packagePath();
                return packageBasePath.startsWith(workspaceDir);
            }));
            const includes = [];
            for (const pkg of filteredPackages) {
                const packageBasePath = yield pkg();
                const include = path.join(packageBasePath, "include");
                if (yield promisifiedExists(include)) {
                    includes.push(include);
                }
            }
            return includes;
        });
    }
    findPackageExecutables(packageName) {
        let command;
        if (process.platform === "win32") {
            return this._findPackageFiles(packageName, `--libexec`, `*.exe`);
        }
        else {
            const dirs = `catkin_find --without-underlays --libexec --share '${packageName}'`;
            command = `find -L $(${dirs}) -type f -executable`;
            return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) => err ? e(err) : c(out.trim().split(os.EOL))));
        }
    }
    findPackageLaunchFiles(packageName) {
        let command;
        if (process.platform === "win32") {
            return this._findPackageFiles(packageName, `--share`, `*.launch`);
        }
        else {
            const dirs = `catkin_find --without-underlays --share '${packageName}'`;
            command = `find -L $(${dirs}) -type f -name *.launch`;
        }
        return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) => {
            err ? e(err) : c(out.trim().split(os.EOL));
        }));
    }
    findPackageTestFiles(packageName) {
        let command;
        if (process.platform === "win32") {
            return this._findPackageFiles(packageName, `--share`, `*.test`);
        }
        else {
            const dirs = `catkin_find --without-underlays --share '${packageName}'`;
            command = `find -L $(${dirs}) -type f -name *.test`;
        }
        return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) => {
            err ? e(err) : c(out.trim().split(os.EOL));
        }));
    }
    startCore() {
        if (typeof this.env.ROS_MASTER_URI === "undefined") {
            return;
        }
        ros_core.startCore(this.context);
    }
    stopCore() {
        if (typeof this.env.ROS_MASTER_URI === "undefined") {
            return;
        }
        ros_core.stopCore(this.context, this._getXmlRpcApi());
    }
    getCoreStatus() {
        return this._getXmlRpcApi().check();
    }
    activateCoreMonitor() {
        if (typeof this.env.ROS_MASTER_URI === "undefined") {
            return null;
        }
        const coreStatusItem = new ros_core.StatusBarItem(this._getXmlRpcApi());
        coreStatusItem.activate();
        return coreStatusItem;
    }
    rosdep() {
        const terminal = ros_utils.createTerminal(this.context);
        this.setTerminalEnv(terminal, this.env);
        terminal.sendText(`rosdep install --from-paths src --ignore-src -r -y`);
        return terminal;
    }
    showCoreMonitor() {
        ros_core.launchMonitor(this.context);
    }
    activateRosrun(packageName, executableName, argument) {
        const terminal = ros_utils.createTerminal(this.context);
        this.setTerminalEnv(terminal, this.env);
        terminal.sendText(`rosrun ${packageName} ${executableName} ${argument}`);
        return terminal;
    }
    activateRoslaunch(launchFilepath, argument) {
        const terminal = ros_utils.createTerminal(this.context);
        this.setTerminalEnv(terminal, this.env);
        terminal.sendText(`roslaunch ${launchFilepath} ${argument}`);
        return terminal;
    }
    activateRostest(launchFilepath, argument) {
        const terminal = ros_utils.createTerminal(this.context);
        this.setTerminalEnv(terminal, this.env);
        terminal.sendText(`rostest ${launchFilepath} ${argument}`);
        return terminal;
    }
    setTerminalEnv(terminal, env) {
        if (process.platform === "linux") {
            for (var item in env) {
                terminal.sendText(`export ${item}=${env[item]} >/dev/null`);
            }
        }
    }
    _findPackageFiles(packageName, filter, pattern) {
        return new Promise((c, e) => child_process.exec(`catkin_find --without-underlays ${filter} ${packageName}`, { env: this.env }, (err, out) => {
            const findFilePromises = [];
            const paths = out.trim().split(os.EOL);
            paths.forEach((foundPath) => {
                const normalizedPath = path.win32.normalize(foundPath);
                findFilePromises.push(new Promise((found) => child_process.exec(`where /r "${normalizedPath}" ` + pattern, { env: this.env }, (innerErr, innerOut) => innerErr ? found(null) : found(innerOut.trim().split(os.EOL)))));
            });
            return Promise.all(findFilePromises).then((values) => {
                // remove null elements
                values = values.filter((s) => s != null);
                // flatten
                values = [].concat(...values);
                c(values);
            });
        }));
    }
    _getXmlRpcApi() {
        if (this.xmlRpcApi === null) {
            this.xmlRpcApi = new ros_core.XmlRpcApi(this.env.ROS_MASTER_URI);
        }
        return this.xmlRpcApi;
    }
}
exports.ROS1 = ROS1;
//# sourceMappingURL=ros1.js.map