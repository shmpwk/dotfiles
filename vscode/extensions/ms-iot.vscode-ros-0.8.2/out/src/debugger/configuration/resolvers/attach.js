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
exports.AttachResolver = void 0;
const vscode = require("vscode");
const child_process = require("child_process");
const os = require("os");
const port_finder = require("portfinder");
const sudo = require("sudo-prompt");
const util = require("util");
const extension = require("../../../extension");
const process_picker = require("../../process-picker/process-picker");
const picker_items_provider_factory = require("../../process-picker/process-items-provider");
const utils = require("../../utils");
const promisifiedExec = util.promisify(child_process.exec);
const promisifiedSudoExec = util.promisify((command, options, cb) => sudo.exec(command, options, (error, stdout, stderr) => cb(error, stdout)));
class AttachResolver {
    constructor() {
        this.supportedRuntimeTypes = [
            "C++",
            "Python",
        ];
    }
    resolveDebugConfigurationWithSubstitutedVariables(folder, config, token) {
        return __awaiter(this, void 0, void 0, function* () {
            // ${command:} variables only get resolved before passed to debug adapter, need to be manually resolve here
            // all ${action:} variables need to be resolved before our resolver propagates the configuration to actual debugger
            yield this.resolveRuntimeIfNeeded(this.supportedRuntimeTypes, config);
            yield this.resolveProcessIdIfNeeded(config);
            yield this.resolveCommandLineIfNeeded(config);
            // propagate debug configuration to Python or C++ debugger depending on the chosen runtime type
            this.launchAttachSession(config);
            // Return null as we have spawned new debug session
            return null;
        });
    }
    launchAttachSession(config) {
        return __awaiter(this, void 0, void 0, function* () {
            if (!config.runtime || !config.processId) {
                return;
            }
            let debugConfig;
            if (config.runtime === "C++") {
                if (os.platform() === "win32") {
                    const cppvsdbgAttachConfig = {
                        name: `C++: ${config.processId}`,
                        type: "cppvsdbg",
                        request: "attach",
                        processId: config.processId,
                    };
                    debugConfig = cppvsdbgAttachConfig;
                }
                else {
                    const cppdbgAttachConfig = {
                        name: `C++: ${config.processId}`,
                        type: "cppdbg",
                        request: "attach",
                        program: config.commandLine,
                        processId: config.processId,
                    };
                    debugConfig = cppdbgAttachConfig;
                }
            }
            else if (config.runtime === "Python") {
                const host = "localhost";
                const port = yield port_finder.getPortPromise();
                const ptvsdInjectCommand = yield utils.getPtvsdInjectCommand(host, port, config.processId);
                try {
                    if (os.platform() === "win32") {
                        const processOptions = {
                            cwd: extension.baseDir,
                            env: yield extension.resolvedEnv(),
                        };
                        // "ptvsd --pid" works with child_process.exec() on Windows
                        const result = yield promisifiedExec(ptvsdInjectCommand, processOptions);
                    }
                    else {
                        const processOptions = {
                            name: "ptvsd",
                        };
                        // "ptvsd --pid" requires elevated permission on Ubuntu
                        const result = yield promisifiedSudoExec(ptvsdInjectCommand, processOptions);
                    }
                }
                catch (error) {
                    const errorMsg = `Command [${ptvsdInjectCommand}] failed!`;
                    throw (new Error(errorMsg));
                }
                let statusMsg = `New ptvsd instance running on ${host}:${port} `;
                statusMsg += `injected into process [${config.processId}].` + os.EOL;
                statusMsg += `To re-attach to process [${config.processId}] after disconnecting, `;
                statusMsg += `please create a separate Python remote attach debug configuration `;
                statusMsg += `that uses the host and port listed above.`;
                extension.outputChannel.appendLine(statusMsg);
                extension.outputChannel.show(true);
                vscode.window.showInformationMessage(statusMsg);
                const pythonattachdebugconfiguration = {
                    name: `Python: ${config.processId}`,
                    type: "python",
                    request: "attach",
                    port: port,
                    host: host,
                };
                debugConfig = pythonattachdebugconfiguration;
            }
            if (!debugConfig) {
                return;
            }
            const launched = yield vscode.debug.startDebugging(undefined, debugConfig);
            if (!launched) {
                throw (new Error(`Failed to start debug session!`));
            }
        });
    }
    resolveRuntimeIfNeeded(supportedRuntimeTypes, config) {
        return __awaiter(this, void 0, void 0, function* () {
            if (config.runtime && config.runtime !== "${action:pick}") {
                return;
            }
            const chooseRuntimeOptions = {
                placeHolder: "Choose runtime type of node to attach to.",
            };
            config.runtime = yield vscode.window.showQuickPick(supportedRuntimeTypes, chooseRuntimeOptions).then((runtime) => {
                if (!runtime) {
                    throw new Error("Runtime type not chosen!");
                }
                return runtime;
            });
        });
    }
    resolveProcessIdIfNeeded(config) {
        return __awaiter(this, void 0, void 0, function* () {
            if (config.processId && config.processId !== "${action:pick}") {
                return;
            }
            const processItemsProvider = picker_items_provider_factory.LocalProcessItemsProviderFactory.Get();
            const processPicker = new process_picker.LocalProcessPicker(processItemsProvider);
            const process = yield processPicker.pick();
            config.processId = process.pid;
        });
    }
    resolveCommandLineIfNeeded(config) {
        return __awaiter(this, void 0, void 0, function* () {
            // this step is only needed on Ubuntu when user has specified PID of C++ executable to attach to
            if (os.platform() === "win32" || config.commandLine || config.runtime !== "C++") {
                return;
            }
            if (!config.processId) {
                throw (new Error("No PID specified!"));
            }
            try {
                const result = yield promisifiedExec(`ls -l /proc/${config.processId}/exe`);
                // contains a space
                const searchTerm = "-> ";
                const indexOfFirst = result.stdout.indexOf(searchTerm);
                config.commandLine = result.stdout.substring(indexOfFirst + searchTerm.length).trim();
            }
            catch (error) {
                throw (new Error(`Failed to resolve command line for process [${config.processId}]!`));
            }
        });
    }
}
exports.AttachResolver = AttachResolver;
//# sourceMappingURL=attach.js.map