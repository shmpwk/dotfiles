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
var __asyncValues = (this && this.__asyncValues) || function (o) {
    if (!Symbol.asyncIterator) throw new TypeError("Symbol.asyncIterator is not defined.");
    var m = o[Symbol.asyncIterator], i;
    return m ? m.call(o) : (o = typeof __values === "function" ? __values(o) : o[Symbol.iterator](), i = {}, verb("next"), verb("throw"), verb("return"), i[Symbol.asyncIterator] = function () { return this; }, i);
    function verb(n) { i[n] = o[n] && function (v) { return new Promise(function (resolve, reject) { v = o[n](v), settle(resolve, reject, v.done, v.value); }); }; }
    function settle(resolve, reject, d, v) { Promise.resolve(v).then(function(v) { resolve({ value: v, done: d }); }, reject); }
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.isApplicable = exports.ColconProvider = void 0;
const vscode = require("vscode");
const path = require("path");
const child_process = require("child_process");
const extension = require("../extension");
const rosShell = require("./ros-shell");
function makeColcon(command, verb, args, category) {
    const task = rosShell.make({ type: command, command, args: [verb, '--symlink-install', '--event-handlers', 'console_cohesion+', '--base-paths', extension.baseDir, `--cmake-args`, `-DCMAKE_BUILD_TYPE=RelWithDebInfo`, ...args] }, category);
    task.problemMatchers = ["$catkin-gcc"];
    return task;
}
/**
 * Provides colcon build and test tasks.
 */
class ColconProvider {
    provideTasks(token) {
        const make = makeColcon('colcon', 'build', [], 'build');
        make.group = vscode.TaskGroup.Build;
        const test = makeColcon('colcon', 'test', [], 'test');
        test.group = vscode.TaskGroup.Test;
        return [make, test];
    }
    resolveTask(task, token) {
        return rosShell.resolve(task);
    }
}
exports.ColconProvider = ColconProvider;
function isApplicable(dir) {
    var e_1, _a;
    return __awaiter(this, void 0, void 0, function* () {
        let colconCommand;
        const srcDir = path.join(dir, "src");
        if (process.platform === "win32") {
            colconCommand = `colcon --log-base nul list --base-paths \"${srcDir}\"`;
        }
        else {
            colconCommand = `colcon --log-base /dev/null list --base-paths ${srcDir}`;
        }
        const { stdout, stderr } = yield child_process.exec(colconCommand);
        try {
            // Does this workspace have packages?
            for (var stdout_1 = __asyncValues(stdout), stdout_1_1; stdout_1_1 = yield stdout_1.next(), !stdout_1_1.done;) {
                const line = stdout_1_1.value;
                // Yes.
                return true;
            }
        }
        catch (e_1_1) { e_1 = { error: e_1_1 }; }
        finally {
            try {
                if (stdout_1_1 && !stdout_1_1.done && (_a = stdout_1.return)) yield _a.call(stdout_1);
            }
            finally { if (e_1) throw e_1.error; }
        }
        // no.
        return false;
    });
}
exports.isApplicable = isApplicable;
//# sourceMappingURL=colcon.js.map