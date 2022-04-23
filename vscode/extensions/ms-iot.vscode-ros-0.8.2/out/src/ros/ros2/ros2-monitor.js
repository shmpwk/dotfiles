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
Object.defineProperty(exports, "__esModule", { value: true });
exports.XmlRpcApi = exports.launchMonitor = void 0;
const path = require("path");
const vscode = require("vscode");
const xmlrpc = require("xmlrpc");
const extension = require("../../extension");
const telemetry = require("../../telemetry-helper");
function getDaemonPort() {
    let basePort = 11511;
    return basePort;
}
function getDaemonUri() {
    return `http://localhost:${getDaemonPort()}/ros2cli/`;
}
function launchMonitor(context) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.ShowCoreStatus);
    const panel = vscode.window.createWebviewPanel("ros2Status", "ROS2 Status", vscode.ViewColumn.Two, {
        enableScripts: true,
    });
    const stylesheet = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "assets", "ros", "core-monitor", "style.css")));
    const script = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "out", "src", "ros", "ros2", "webview", "main.js")));
    panel.webview.html = getCoreStatusWebviewContent(stylesheet, script);
    const ros2cliApi = new XmlRpcApi();
    const pollingHandle = setInterval(() => __awaiter(this, void 0, void 0, function* () {
        try {
            const result = yield Promise.all([
                ros2cliApi.getNodeNamesAndNamespaces(),
                ros2cliApi.getTopicNamesAndTypes(),
                ros2cliApi.getServiceNamesAndTypes()
            ]);
            const nodesJSON = JSON.stringify(result[0]);
            const topicsJSON = JSON.stringify(result[1]);
            const servicesJSON = JSON.stringify(result[2]);
            panel.webview.postMessage({
                ready: true,
                nodes: nodesJSON,
                topics: topicsJSON,
                services: servicesJSON,
            });
        }
        catch (e) {
            panel.webview.postMessage({
                ready: false,
            });
        }
    }), 200);
    panel.onDidDispose(() => {
        clearInterval(pollingHandle);
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
    <h1>ROS2 System Status</h1>
    <h2 id="ros-status">-</h2>

    <div id="parameters"></div>
    <div id="topics"></div>
    <div id="services"></div>
</body>

</html>
`;
}
/**
 * ros2cli xmlrpc interfaces.
 */
class XmlRpcApi {
    constructor() {
        this.client = xmlrpc.createClient(getDaemonUri());
    }
    check() {
        // the ROS2 CLI doesn't have an API which returns detailed status, 
        // so we're just using another endpoint to verify it is running
        return this.methodCall("get_node_names_and_namespaces").then(() => true, () => false);
    }
    getNodeNamesAndNamespaces() {
        return this.methodCall("get_node_names_and_namespaces");
    }
    getServiceNamesAndTypes() {
        return this.methodCall("get_service_names_and_types");
    }
    getTopicNamesAndTypes() {
        return this.methodCall("get_topic_names_and_types");
    }
    methodCall(method, ...args) {
        return new Promise((resolve, reject) => {
            this.client.methodCall(method, [...args], (err, val) => {
                if (err) {
                    reject(err);
                }
                else {
                    resolve(val);
                }
            });
        });
    }
}
exports.XmlRpcApi = XmlRpcApi;
//# sourceMappingURL=ros2-monitor.js.map