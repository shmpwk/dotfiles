"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.execChildProcess = void 0;
// copied from https://github.com/microsoft/vscode-cpptools
/* tslint:disable */
const child_process = require("child_process");
function execChildProcess(process, workingDirectory) {
    return new Promise((resolve, reject) => {
        child_process.exec(process, { cwd: workingDirectory, maxBuffer: 500 * 1024 }, (error, stdout, stderr) => {
            if (error) {
                reject(error);
                return;
            }
            if (stderr && stderr.length > 0 && !stderr.includes('screen size is bogus')) {
                reject(new Error(stderr));
                return;
            }
            resolve(stdout);
        });
    });
}
exports.execChildProcess = execChildProcess;
//# sourceMappingURL=utils.js.map