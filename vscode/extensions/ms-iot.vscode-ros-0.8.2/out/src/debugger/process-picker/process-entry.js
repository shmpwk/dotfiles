"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.createProcessQuickPickItem = exports.ProcessEntry = void 0;
class ProcessEntry {
    // constructor(readonly name: string, readonly pid: string, readonly commandLine: string) {}
    constructor(name, pid, commandLine) {
        this.name = name;
        this.pid = pid;
        this.commandLine = commandLine;
    }
}
exports.ProcessEntry = ProcessEntry;
function createProcessQuickPickItem(entry) {
    return {
        label: entry.name,
        description: entry.pid,
        detail: entry.commandLine,
        pid: entry.pid,
    };
}
exports.createProcessQuickPickItem = createProcessQuickPickItem;
//# sourceMappingURL=process-entry.js.map