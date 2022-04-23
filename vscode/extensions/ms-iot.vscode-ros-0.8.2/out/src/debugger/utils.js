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
exports.getPtvsdInjectCommand = exports.getDebugSettings = exports.oneTimePromiseFromEvent = void 0;
const vscode = require("vscode");
const extension = require("../extension");
const telemetry = require("../telemetry-helper");
function oneTimePromiseFromEvent(eventCall, filter = undefined) {
    return __awaiter(this, void 0, void 0, function* () {
        return new Promise(resolve => {
            let disposable;
            disposable = eventCall(event => {
                if (filter && !filter(event)) {
                    return;
                }
                disposable.dispose();
                resolve(event);
            });
        });
    });
}
exports.oneTimePromiseFromEvent = oneTimePromiseFromEvent;
/**
 * Gets stringified settings to pass to the debug server.
 */
function getDebugSettings(context) {
    return __awaiter(this, void 0, void 0, function* () {
        const reporter = telemetry.getReporter();
        reporter.sendTelemetryCommand(extension.Commands.GetDebugSettings);
        return JSON.stringify({ env: extension.env });
    });
}
exports.getDebugSettings = getDebugSettings;
function getPtvsdInjectCommand(host, port, pid) {
    return __awaiter(this, void 0, void 0, function* () {
        // instead of requiring presence of correctly versioned ptvsd from pip (https://github.com/Microsoft/ptvsd)
        // use ptvsd shipped with vscode-python to avoid potential version mismatch
        const pyExtensionId = "ms-python.python";
        const pyExtension = vscode.extensions.getExtension(pyExtensionId);
        if (pyExtension) {
            if (!pyExtension.isActive) {
                yield pyExtension.activate();
            }
            // tslint:disable-next-line:strict-boolean-expressions
            if (pyExtension.exports && pyExtension.exports.debug) {
                // hack python extension's api to get command for injecting ptvsd
                // pass false for waitForDebugger so the --wait flag won't be added
                const waitForDebugger = false;
                const ptvsdCommand = yield pyExtension.exports.debug.getRemoteLauncherCommand(host, port, waitForDebugger);
                // prepend python interpreter
                ptvsdCommand.unshift("python");
                // append the --pid flag
                ptvsdCommand.push("--pid", pid.toString());
                return ptvsdCommand.join(" ");
            }
            else {
                throw new Error(`Update extension [${pyExtensionId}] to debug Python projects.`);
            }
        }
        throw new Error("Failed to retrieve ptvsd from Python extension!");
    });
}
exports.getPtvsdInjectCommand = getPtvsdInjectCommand;
//# sourceMappingURL=utils.js.map