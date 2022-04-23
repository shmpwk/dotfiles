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
exports.createPackage = exports.CatkinMakeIsolatedProvider = exports.CatkinMakeProvider = void 0;
const vscode = require("vscode");
const extension = require("../extension");
const common = require("./common");
const rosShell = require("./ros-shell");
function makeCatkin(command, args, category) {
    const task = rosShell.make({ type: command, command, args: ['--directory', extension.baseDir, '-DCMAKE_BUILD_TYPE=RelWithDebInfo', ...args] }, category);
    task.problemMatchers = ["$catkin-gcc"];
    return task;
}
/**
 * Provides catkin_make build and test tasks
 */
class CatkinMakeProvider {
    provideTasks(token) {
        const make = makeCatkin('catkin_make', [], 'build');
        make.group = vscode.TaskGroup.Build;
        const test = makeCatkin('catkin_make', ['run_tests'], 'run_tests');
        test.group = vscode.TaskGroup.Test;
        return [make, test];
    }
    resolveTask(task, token) {
        return rosShell.resolve(task);
    }
}
exports.CatkinMakeProvider = CatkinMakeProvider;
/**
 * Provides catkin_make_isolated build and test tasks
 */
class CatkinMakeIsolatedProvider {
    provideTasks(token) {
        const make = makeCatkin('catkin_make_isolated', [], 'build');
        make.group = vscode.TaskGroup.Build;
        const test = makeCatkin('catkin_make_isolated', ['--catkin-make-args', 'run_tests'], 'run_tests');
        test.group = vscode.TaskGroup.Test;
        return [make, test];
    }
    resolveTask(task, token) {
        return rosShell.resolve(task);
    }
}
exports.CatkinMakeIsolatedProvider = CatkinMakeIsolatedProvider;
/**
 * Interacts with the user to run a `catkin_create_pkg` command.
 */
function createPackage(uri) {
    return __awaiter(this, void 0, void 0, function* () {
        const createPkgCommand = (dependencies, name) => {
            return `catkin_create_pkg ${name} ${dependencies}`;
        };
        return common._createPackage(createPkgCommand);
    });
}
exports.createPackage = createPackage;
//# sourceMappingURL=catkin.js.map