"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.make = exports.resolve = exports.registerRosShellTaskProvider = exports.RosShellTaskProvider = void 0;
const vscode = require("vscode");
const extension = require("../extension");
class RosShellTaskProvider {
    provideTasks(token) {
        return this.defaultRosTasks();
    }
    defaultRosTasks() {
        const rosCore = make({ type: 'ros', command: 'roscore' }, 'roscore');
        rosCore.isBackground = true;
        rosCore.problemMatchers = ['$roscore'];
        const rosLaunch = make({ type: 'ros', command: 'roslaunch', args: ['package_name', 'launch_file.launch'] }, 'roslaunch');
        rosLaunch.isBackground = true;
        rosLaunch.problemMatchers = ['$roslaunch'];
        return [rosCore, rosLaunch];
    }
    resolveTask(task, token) {
        return resolve(task);
    }
}
exports.RosShellTaskProvider = RosShellTaskProvider;
function registerRosShellTaskProvider() {
    return [
        vscode.tasks.registerTaskProvider('ros', new RosShellTaskProvider()),
    ];
}
exports.registerRosShellTaskProvider = registerRosShellTaskProvider;
function resolve(task) {
    const resolvedTask = make(task.definition);
    resolvedTask.isBackground = task.isBackground;
    resolvedTask.problemMatchers = task.problemMatchers;
    return resolvedTask;
}
exports.resolve = resolve;
function make(definition, category) {
    definition.command = definition.command || definition.type; // Command can be missing in build tasks that have type==command
    const args = definition.args || [];
    const name = category ? category : args.join(' ');
    const task = new vscode.Task(definition, vscode.TaskScope.Workspace, name, definition.command);
    task.execution = new vscode.ShellExecution(definition.command, args, {
        env: extension.env,
    });
    return task;
}
exports.make = make;
//# sourceMappingURL=ros-shell.js.map