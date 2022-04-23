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
exports.createPackage = exports.CatkinToolsProvider = void 0;
const vscode = require("vscode");
const extension = require("../extension");
const common = require("./common");
const rosShell = require("./ros-shell");
function makeCatkin(command, args, category) {
    const task = rosShell.make({ type: command, command, args: ['--workspace', extension.baseDir, ...args] }, category);
    task.problemMatchers = ["$catkin-gcc"];
    return task;
}
/**
 * Provides catkin tools build and test tasks.
 */
class CatkinToolsProvider {
    provideTasks(token) {
        const make = makeCatkin('catkin', [], 'build');
        make.group = vscode.TaskGroup.Build;
        const test = makeCatkin('catkin', ['--catkin-make-args', 'run_tests'], 'run_tests');
        test.group = vscode.TaskGroup.Test;
        return [make, test];
    }
    resolveTask(task, token) {
        return rosShell.resolve(task);
    }
}
exports.CatkinToolsProvider = CatkinToolsProvider;
/**
 * Interacts with the user to run a `catkin create pkg` command.
 */
function createPackage(uri) {
    return __awaiter(this, void 0, void 0, function* () {
        const createPkgCommand = (dependencies, name) => {
            return `catkin create pkg --catkin-deps ${dependencies} -- ${name}`;
        };
        return common._createPackage(createPkgCommand);
    });
}
exports.createPackage = createPackage;
//# sourceMappingURL=catkin-tools.js.map