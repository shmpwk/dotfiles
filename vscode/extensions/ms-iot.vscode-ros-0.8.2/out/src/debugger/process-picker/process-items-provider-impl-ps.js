"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.PsProcessParser = exports.PsProvider = void 0;
// copied from https://github.com/microsoft/vscode-cpptools
/* tslint:disable */
const os = require("os");
const process_item = require("./process-entry");
const local_provider = require("./local-process-items-provider");
const utils = require("./utils");
class PsProvider extends local_provider.LocalProcessQuickPickItemsProvider {
    // Perf numbers:
    // OS X 10.10
    // | # of processes | Time (ms) |
    // |----------------+-----------|
    // |            272 |        52 |
    // |            296 |        49 |
    // |            384 |        53 |
    // |            784 |       116 |
    //
    // Ubuntu 16.04
    // | # of processes | Time (ms) |
    // |----------------+-----------|
    // |            232 |        26 |
    // |            336 |        34 |
    // |            736 |        62 |
    // |           1039 |       115 |
    // |           1239 |       182 |
    // ps outputs as a table. With the option "ww", ps will use as much width as necessary.
    // However, that only applies to the right-most column. Here we use a hack of setting
    // the column header to 50 a's so that the second column will have at least that many
    // characters. 50 was chosen because that's the maximum length of a "label" in the
    // QuickPick UI in VSCode.
    getInternalProcessEntries() {
        let processCmd = '';
        switch (os.platform()) {
            case 'darwin':
                processCmd = PsProcessParser.psDarwinCommand;
                break;
            case 'linux':
                processCmd = PsProcessParser.psLinuxCommand;
                break;
            default:
                return Promise.reject(new Error(`Operating system "${os.platform()}" not support.`));
        }
        return utils.execChildProcess(processCmd, null).then((processes) => {
            return PsProcessParser.ParseProcessFromPs(processes);
        });
    }
}
exports.PsProvider = PsProvider;
// tslint:disable-next-line: max-classes-per-file
class PsProcessParser {
    static get secondColumnCharacters() { return 50; }
    static get commColumnTitle() { return Array(PsProcessParser.secondColumnCharacters).join("a"); }
    // the BSD version of ps uses '-c' to have 'comm' only output the executable name and not
    // the full path. The Linux version of ps has 'comm' to only display the name of the executable
    // Note that comm on Linux systems is truncated to 16 characters:
    // https://bugzilla.redhat.com/show_bug.cgi?id=429565
    // Since 'args' contains the full path to the executable, even if truncated, searching will work as desired.
    static get psLinuxCommand() {
        return `ps axww -o pid=,comm=${PsProcessParser.commColumnTitle},args=`;
    }
    static get psDarwinCommand() {
        return `ps axww -o pid=,comm=${PsProcessParser.commColumnTitle},args= -c`;
    }
    // Only public for tests.
    static ParseProcessFromPs(processes) {
        let lines = processes.split(os.EOL);
        return PsProcessParser.ParseProcessFromPsArray(lines);
    }
    static ParseProcessFromPsArray(processArray) {
        let processEntries = [];
        // lines[0] is the header of the table
        for (let i = 1; i < processArray.length; i++) {
            let line = processArray[i];
            if (!line) {
                continue;
            }
            let processEntry = PsProcessParser.parseLineFromPs(line);
            processEntries.push(processEntry);
        }
        return processEntries;
    }
    static parseLineFromPs(line) {
        // Explanation of the regex:
        //   - any leading whitespace
        //   - PID
        //   - whitespace
        //   - executable name --> this is PsAttachItemsProvider.secondColumnCharacters - 1 because ps reserves one character
        //     for the whitespace separator
        //   - whitespace
        //   - args (might be empty)
        const psEntry = new RegExp(`^\\s*([0-9]+)\\s+(.{${PsProcessParser.secondColumnCharacters - 1}})\\s+(.*)$`);
        const matches = psEntry.exec(line);
        if (matches && matches.length === 4) {
            const pid = matches[1].trim();
            const executable = matches[2].trim();
            const cmdline = matches[3].trim();
            return new process_item.ProcessEntry(executable, pid, cmdline);
        }
    }
}
exports.PsProcessParser = PsProcessParser;
//# sourceMappingURL=process-items-provider-impl-ps.js.map