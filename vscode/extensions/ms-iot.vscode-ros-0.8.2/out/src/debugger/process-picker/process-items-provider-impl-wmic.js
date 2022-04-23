"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.WmicProcessParser = exports.WmicProvider = void 0;
// copied from https://github.com/microsoft/vscode-cpptools
/* tslint:disable */
const os = require("os");
const native_provider = require("./local-process-items-provider");
const process_item = require("./process-entry");
const utils = require("./utils");
class WmicProvider extends native_provider.LocalProcessQuickPickItemsProvider {
    // Perf numbers on Win10:
    // | # of processes | Time (ms) |
    // |----------------+-----------|
    // |            309 |       413 |
    // |            407 |       463 |
    // |            887 |       746 |
    // |           1308 |      1132 |
    getInternalProcessEntries() {
        const wmicCommand = 'wmic process get Name,ProcessId,CommandLine /FORMAT:list';
        return utils.execChildProcess(wmicCommand, null).then((processes) => {
            return WmicProcessParser.ParseProcessFromWmic(processes);
        });
    }
}
exports.WmicProvider = WmicProvider;
// tslint:disable-next-line: max-classes-per-file
class WmicProcessParser {
    static get wmicNameTitle() { return 'Name'; }
    static get wmicCommandLineTitle() { return 'CommandLine'; }
    static get wmicPidTitle() { return 'ProcessId'; }
    // Only public for tests.
    static ParseProcessFromWmic(processes) {
        let lines = processes.split(os.EOL);
        let currentProcess = new process_item.ProcessEntry(null, null, null);
        let processEntries = [];
        for (let i = 0; i < lines.length; i++) {
            let line = lines[i];
            if (!line) {
                continue;
            }
            WmicProcessParser.parseLineFromWmic(line, currentProcess);
            // Each entry of processes has ProcessId as the last line
            if (line.lastIndexOf(WmicProcessParser.wmicPidTitle, 0) === 0) {
                processEntries.push(currentProcess);
                currentProcess = new process_item.ProcessEntry(null, null, null);
            }
        }
        return processEntries;
    }
    static parseLineFromWmic(line, process) {
        let splitter = line.indexOf('=');
        if (splitter >= 0) {
            let key = line.slice(0, line.indexOf('=')).trim();
            let value = line.slice(line.indexOf('=') + 1).trim();
            if (key === WmicProcessParser.wmicNameTitle) {
                process.name = value;
            }
            else if (key === WmicProcessParser.wmicPidTitle) {
                process.pid = value;
            }
            else if (key === WmicProcessParser.wmicCommandLineTitle) {
                const extendedLengthPath = '\\??\\';
                if (value.lastIndexOf(extendedLengthPath, 0) === 0) {
                    value = value.slice(extendedLengthPath.length);
                }
                process.commandLine = value;
            }
        }
    }
}
exports.WmicProcessParser = WmicProcessParser;
//# sourceMappingURL=process-items-provider-impl-wmic.js.map