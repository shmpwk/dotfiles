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
exports.clearReporter = exports.getReporter = void 0;
const vscode_extension_telemetry_1 = require("vscode-extension-telemetry");
const vscode_utils = require("./vscode-utils");
let reporterSingleton;
function getTelemetryReporter() {
    if (reporterSingleton) {
        return reporterSingleton;
    }
    const extensionId = "ms-iot.vscode-ros";
    const packageInfo = vscode_utils.getPackageInfo(extensionId);
    if (packageInfo) {
        reporterSingleton = new vscode_extension_telemetry_1.default(packageInfo.name, packageInfo.version, packageInfo.aiKey);
    }
    return reporterSingleton;
}
var TelemetryEvent;
(function (TelemetryEvent) {
    TelemetryEvent["activate"] = "activate";
    TelemetryEvent["command"] = "command";
})(TelemetryEvent || (TelemetryEvent = {}));
class SimpleReporter {
    constructor() {
        this.telemetryReporter = getTelemetryReporter();
    }
    sendTelemetryActivate() {
        if (!this.telemetryReporter) {
            return;
        }
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.activate);
    }
    sendTelemetryCommand(commandName) {
        if (!this.telemetryReporter) {
            return;
        }
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.command, {
            name: commandName,
        });
    }
}
function getReporter() {
    return (new SimpleReporter());
}
exports.getReporter = getReporter;
function clearReporter() {
    return __awaiter(this, void 0, void 0, function* () {
        yield reporterSingleton.dispose();
        reporterSingleton = undefined;
    });
}
exports.clearReporter = clearReporter;
//# sourceMappingURL=telemetry-helper.js.map