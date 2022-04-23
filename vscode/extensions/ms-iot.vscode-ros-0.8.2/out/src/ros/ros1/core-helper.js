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
exports.StatusBarItem = exports.XmlRpcApi = exports.launchMonitor = exports.stopCore = exports.startCore = void 0;
const child_process = require("child_process");
const path = require("path");
const vscode = require("vscode");
const xmlrpc = require("xmlrpc");
const extension = require("../../extension");
const telemetry = require("../../telemetry-helper");
function startCore(context) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.StartRosCore);
    let launchCoreCommand = "roscore";
    let processOptions = {
        cwd: extension.baseDir,
        env: extension.env,
    };
    const roscoreProcess = child_process.spawn(launchCoreCommand, processOptions);
    roscoreProcess.on('error', (_err) => {
        vscode.window.showErrorMessage("Failed to launch ROS core.");
    });
}
exports.startCore = startCore;
function stopCore(context, api) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.TerminateRosCore);
    if (process.platform === "win32") {
        api.getPid().then(pid => child_process.exec(`taskkill /pid ${pid} /f`));
    }
    else {
        api.getPid().then(pid => child_process.exec(`kill $(ps -o ppid= -p '${pid}')`));
    }
}
exports.stopCore = stopCore;
function launchMonitor(context) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.ShowCoreStatus);
    const panel = vscode.window.createWebviewPanel("rosCoreStatus", "ROS Core Status", vscode.ViewColumn.Two, {
        enableScripts: true,
    });
    let stylesheet = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "assets", "ros", "core-monitor", "style.css")));
    let script = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "out", "src", "ros", "ros1", "core-monitor", "main.js")));
    panel.webview.html = getCoreStatusWebviewContent(stylesheet, script);
    const pollingStatus = setInterval(() => {
        const masterApi = new XmlRpcApi(extension.env.ROS_MASTER_URI);
        masterApi.check().then((status) => {
            if (status) {
                let getParameters = masterApi.getParam("/");
                let getSystemState = masterApi.getSystemState();
                Promise.all([getParameters, getSystemState]).then(([parameters, systemState]) => {
                    let parametersJSON = JSON.stringify(parameters);
                    let systemStateJSON = JSON.stringify(systemState);
                    panel.webview.postMessage({
                        status: status,
                        parameters: parametersJSON,
                        systemState: systemStateJSON,
                    });
                });
            }
            else {
                panel.webview.postMessage({
                    status: status,
                });
            }
        });
    }, 100);
    panel.onDidDispose(() => {
        clearInterval(pollingStatus);
    });
}
exports.launchMonitor = launchMonitor;
function getCoreStatusWebviewContent(stylesheet, script) {
    return `
<!DOCTYPE html>
<html lang="en">

<head>
    <link rel="stylesheet" href="${stylesheet.toString()}" />

    <script src="${script.toString()}"></script>
</head>

<body>
    <h1>ROS Core Status</h1>
    <h2 id="ros-status">-</h2>

    <div id="parameters"></div>
    <div id="topics"></div>
    <div id="services"></div>
</body>

</html>
`;
}
const CALLER_ID = "vscode-ros";
/**
 * Exposes the ROS master XML-RPC api.
 */
class XmlRpcApi {
    constructor(uri) {
        this.client = xmlrpc.createClient(uri);
    }
    /**
     * Returns true if a master process is running.
     */
    check() {
        return this.getPid().then(() => true, () => false);
    }
    getPid() {
        return this.methodCall("getPid");
    }
    getSystemState() {
        const responseReducer = (acc, cur) => {
            const k = cur[0];
            const v = cur[1];
            acc[k] = v;
            return acc;
        };
        return this.methodCall("getSystemState").then((res) => {
            const systemState = {
                publishers: res[0].reduce(responseReducer, {}),
                services: res[2].reduce(responseReducer, {}),
                subscribers: res[1].reduce(responseReducer, {}),
            };
            return systemState;
        });
    }
    getParamNames() {
        return this.methodCall("getParamNames");
    }
    getParam(name) {
        return this.methodCall("getParam", name);
    }
    methodCall(method, ...args) {
        return new Promise((resolve, reject) => {
            this.client.methodCall(method, [CALLER_ID, ...args], (err, val) => {
                if (err) {
                    reject(err);
                }
                else if (val[0] !== 1) {
                    reject(val);
                }
                else {
                    resolve(val[2]);
                }
            });
        });
    }
}
exports.XmlRpcApi = XmlRpcApi;
/**
 * Shows the ROS core status in the status bar.
 */
// tslint:disable-next-line: max-classes-per-file
class StatusBarItem {
    constructor(api) {
        this.api = api;
        this.item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 200);
        const waitIcon = "$(clock)";
        const ros = "ROS";
        this.item.text = `${waitIcon} ${ros}`;
        this.item.command = extension.Commands.ShowCoreStatus;
    }
    activate() {
        this.item.show();
        this.timer = setInterval(() => this.update(), 200);
    }
    dispose() {
        this.item.dispose();
    }
    update() {
        return __awaiter(this, void 0, void 0, function* () {
            const status = yield this.api.check();
            if (status === this.status) {
                return;
            }
            const statusIcon = status ? "$(check)" : "$(x)";
            let ros = "ROS";
            // these environment variables are set by the ros_environment package
            // https://github.com/ros/ros_environment
            const rosVersionChecker = "ROS_VERSION";
            const rosDistroChecker = "ROS_DISTRO";
            if (rosVersionChecker in extension.env && rosDistroChecker in extension.env) {
                const rosVersion = extension.env[rosVersionChecker];
                const rosDistro = extension.env[rosDistroChecker];
                ros += `${rosVersion}.${rosDistro}`;
            }
            else {
                // for older ROS1 installations with outdated ros_environment
                // "rosversion --distro" might be needed
                // ignoring such case for now
            }
            this.item.text = `${statusIcon} ${ros}`;
            this.status = status;
        });
    }
}
exports.StatusBarItem = StatusBarItem;
//# sourceMappingURL=core-helper.js.map